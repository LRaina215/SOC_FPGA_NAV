"""Host prototype: bounded wheel-scale observation tiles with explicit age.

Coverage means measured samples in a tile, not a proof of continuous ground.
No height interpolation or hole filling. Age uses capture time, not received
frame count. Newer data cannot be overwritten by an older delayed frame.
"""
import numpy as np


class SupportCoverage:
    def __init__(self,subdiv=4,n=128,min_frames=2,fresh_us=1_000_000,max_age_us=20_000_000):
        assert subdiv in (1,2,4,8) and n&(n-1)==0
        self.subdiv=subdiv;self.n=n;self.inv=1311*subdiv;self.min_frames=min_frames
        self.fresh_us=fresh_us;self.max_age_us=max_age_us;self.clear()

    def clear(self):
        size=self.n*self.n
        self.gx=np.full(size,-2**30,np.int32);self.gy=self.gx.copy()
        self.stamp=np.full(size,-1,np.int64);self.count=np.zeros(size,np.uint8)
        self.cx=self.cy=0;self.seq=0;self.latest_capture_us=-1

    def update(self,points,valid,cx,cy,capture_us,now_us):
        if capture_us<=self.latest_capture_us:return
        oldx,oldy=self.cx,self.cy
        self.cx=int(cx)*self.subdiv;self.cy=int(cy)*self.subdiv
        # Tagless hardware can only retain overlapping window positions. Clear
        # exposed strips even when a prior visit left the same coordinate here.
        if self.cx!=oldx or self.cy!=oldy:
            X,Y=np.meshgrid(np.arange(self.cx-self.n//2,self.cx+self.n//2),np.arange(self.cy-self.n//2,self.cy+self.n//2))
            new=(X<oldx-self.n//2)|(X>=oldx+self.n//2)|(Y<oldy-self.n//2)|(Y>=oldy+self.n//2)
            a=(Y[new]&(self.n-1))*self.n+(X[new]&(self.n-1));self.count[a]=0;self.stamp[a]=-1
        self.latest_capture_us=capture_us
        p=np.asarray(points);v=np.asarray(valid,dtype=bool).copy()
        ix=(p[:,0]*self.inv)>>16;iy=(p[:,1]*self.inv)>>16
        v&=(ix>=self.cx-self.n//2)&(ix<self.cx+self.n//2)&(iy>=self.cy-self.n//2)&(iy<self.cy+self.n//2)
        ix,iy=ix[v],iy[v];a=(iy&(self.n-1))*self.n+(ix&(self.n-1))
        a,first=np.unique(a,return_index=True);ix,iy=ix[first],iy[first]
        same=(self.gx[a]==ix)&(self.gy[a]==iy)
        newer=(~same)|(capture_us>self.stamp[a]);a,ix,iy,same=a[newer],ix[newer],iy[newer],same[newer]
        recent=same&(capture_us-self.stamp[a]<=self.max_age_us)
        self.count[a]=np.where(recent,np.minimum(self.count[a].astype(int)+1,3),1)
        self.gx[a],self.gy[a],self.stamp[a]=ix,iy,capture_us
        self.seq+=1

    def view(self,now_us):
        x=np.arange(self.cx-self.n//2,self.cx+self.n//2,dtype=np.int64)
        y=np.arange(self.cy-self.n//2,self.cy+self.n//2,dtype=np.int64)
        a=(y[:,None]&(self.n-1))*self.n+(x[None,:]&(self.n-1))
        tagged=(self.gx[a]==x[None,:])&(self.gy[a]==y[:,None])
        age=np.maximum(0,int(now_us)-self.stamp[a]);known=tagged&(self.count[a]>=self.min_frames)&(age<=self.max_age_us)
        cls=np.where(known,np.where(age<=self.fresh_us,1,2),0).astype(np.uint8)
        return cls,age,self.count[a],a

    def snapshot(self,now_us):
        cls,age,count,_=self.view(now_us)
        return dict(cls=cls,age_us=age,cnt=count,cx=self.cx,cy=self.cy,inv=self.inv,seq=self.seq)

    def cell(self,x_mm,y_mm,now_us):
        ix=(int(x_mm)*self.inv)>>16;iy=(int(y_mm)*self.inv)>>16
        if not(self.cx-self.n//2<=ix<self.cx+self.n//2 and self.cy-self.n//2<=iy<self.cy+self.n//2):return None
        a=(iy&(self.n-1))*self.n+(ix&(self.n-1));age=int(now_us)-int(self.stamp[a])
        if self.gx[a]!=ix or self.gy[a]!=iy or self.count[a]<self.min_frames or age<0 or age>self.max_age_us:return None
        return int(self.count[a]),age,age>self.fresh_us

    def masks(self,ix,iy,now_us):
        """Known/fresh masks in row-major4x4 subregions of a coarse cell."""
        assert self.subdiv==4
        known=fresh=0
        for sy in range(4):
            for sx in range(4):
                gx,gy=ix*4+sx,iy*4+sy
                if not(self.cx-64<=gx<self.cx+64 and self.cy-64<=gy<self.cy+64):continue
                a=(gy&127)*128+(gx&127);age=int(now_us)-int(self.stamp[a])
                if self.gx[a]!=gx or self.gy[a]!=gy or self.count[a]<self.min_frames or age<0 or age>self.max_age_us:continue
                bit=1<<(sy*4+sx);known|=bit
                if age<=self.fresh_us:fresh|=bit
        return known,fresh

    def mask_windows(self,now_us):
        cls,*_=self.view(now_us)
        return pack_masks(cls,self.subdiv)


def pack_masks(cls,subdiv=4):
    assert subdiv==4 and cls.shape==(128,128)
    blocks=cls.reshape(32,4,32,4).transpose(0,2,1,3).reshape(32,32,16)
    bits=(1<<np.arange(16,dtype=np.uint32))
    return ((blocks!=0)*bits).sum(axis=2).astype(np.uint16),((blocks==1)*bits).sum(axis=2).astype(np.uint16)


class CompactCoverage:
    """9bits/tile: capture epoch7 + distinct-epoch count2. Tagless rolling.

    A 200ms wall-clock epoch advances during dropped/no lidar frames. Age is an
    upper bound, so no evidence is called fresh longer than its actual TTL.
    32bit timer wrap clears the layer and rejects pre-wrap delayed captures.
    """
    def __init__(self):
        self.n=128;self.subdiv=4;self.inv=5244;self.clear()

    def clear(self):
        self.word=np.zeros(128*128,np.uint16);self.cx=self.cy=0
        self.last_now=None;self.latest_capture=None;self.seq=0

    def advance(self,now_us):
        now=int(now_us)&0xffffffff
        if self.last_now is not None:
            if now<self.last_now or now-self.last_now>20_000_000:
                self.word[:]=0;self.latest_capture=None
        self.last_now=now
        epoch=(now//200_000)&127;age=(epoch-(self.word>>2))&127
        self.word[age>99]=0
        return now,epoch

    def update(self,points,valid,cx,cy,capture_us,now_us):
        now,epoch=self.advance(now_us);capture=int(capture_us)&0xffffffff
        if capture>now or now-capture>1_000_000:return
        if self.latest_capture is not None and capture<=self.latest_capture:return
        self.latest_capture=capture;oldx,oldy=self.cx,self.cy;self.cx=int(cx)*4;self.cy=int(cy)*4
        if self.cx!=oldx or self.cy!=oldy:
            X,Y=np.meshgrid(np.arange(self.cx-64,self.cx+64),np.arange(self.cy-64,self.cy+64))
            new=(X<oldx-64)|(X>=oldx+64)|(Y<oldy-64)|(Y>=oldy+64)
            self.word[(Y[new]&127)*128+(X[new]&127)]=0
        p=np.asarray(points);ix=(p[:,0]*self.inv)>>16;iy=(p[:,1]*self.inv)>>16
        v=np.asarray(valid,dtype=bool)&(ix>=self.cx-64)&(ix<self.cx+64)&(iy>=self.cy-64)&(iy<self.cy+64)
        a=np.unique((iy[v]&127)*128+(ix[v]&127));old=self.word[a];stamp=(capture//200_000)&127
        count=old&3;different=(old>>2)!=stamp
        count=np.where(count==0,1,np.where(different,np.minimum(count+1,3),count))
        self.word[a]=(stamp<<2)|count;self.seq+=1

    def view(self,now_us):
        now,epoch=self.advance(now_us)
        x=np.arange(self.cx-64,self.cx+64);y=np.arange(self.cy-64,self.cy+64)
        a=(y[:,None]&127)*128+(x[None,:]&127);w=self.word[a]
        ticks=(epoch-(w>>2))&127;upper=(ticks.astype(np.int64)+1)*200_000
        cls=np.where((w&3)>=2,np.where(ticks<=4,1,np.where(ticks<=99,2,0)),0).astype(np.uint8)
        return cls,upper,w&3,a

    def mask_windows(self,now_us):return pack_masks(self.view(now_us)[0])
