from pathlib import Path
import re
from decimal import Decimal, ROUND_HALF_UP
source=Path('/home/cyd/fpga/references/4针 OLED显示屏资料/03-0.96OLED横屏程序源码/IIC接口显示屏例程/STM32例程/01-0.96OLED显示屏STM32F103C8T6_IIC例程/HARDWARE/OLED/oledfont.h').read_bytes().decode('gb18030')
m=re.search(r'const unsigned char asc2_1206[^=]*=\s*\{(.*?)\n\};',source,re.S)
body=re.sub(r'/\*.*?\*/|//[^\n]*','',m[1],flags=re.S)
font=[[int(h,16)for h in re.findall(r'0[xX][0-9a-fA-F]+',r)]for r in re.findall(r'\{([^{}]*)\}',body)]
def golden(path,values,error=False,wheels=None,header_override=None):
 gram=[0]*1024
 def text(x,y,s):
  for ch in s:
   glyph=font[ord(ch)-32]
   for row in range(12):
    for col in range(6):
     if glyph[(row//8)*6+col] & (1<<(row%8)):
      gram[((y+row)//8)*128+x+col] |= 1<<((y+row)%8)
   x+=6
 title=('BMI088: ERR 7 g/dps'if error else'BMI088: OK g/dps').ljust(21)
 if wheels is not None:
  left,right=wheels
  title=f'L{Decimal(left)/100:+07.2f} R{Decimal(right)/100:+07.2f} m/s'
 if header_override is not None:title=header_override.ljust(21)
 text(0,0,title)
 for i,(label,value)in enumerate(zip(['AX:','AY:','AZ:','GX:','GY:','GZ:'],values)):
  if i<3:
   physical=(Decimal(value)/Decimal(10920)).quantize(Decimal('0.01'),rounding=ROUND_HALF_UP)
   if physical==0:physical=abs(physical)
   field=' -.--g ' if error else f'{physical:+.2f}g '
  else:
   physical=(Decimal(value)/Decimal('16.384')).quantize(Decimal('0.1'),rounding=ROUND_HALF_UP)
   if physical==0:physical=abs(physical)
   field=' ----.-' if error else f'{physical:+07.1f}'
  text(0 if i<3 else 64,16+(i%3)*16,label+field)
 Path(path).write_text('\n'.join(f'{x:02x}'for x in gram)+'\n')
golden('/tmp/axes_normal.mem',[-32768,32767,1234,-1,0,-23456])
golden('/tmp/axes_error.mem',[0]*6,True)
golden('/tmp/axes_new.mem',[1,-2,300,-400,5000,-6000])

golden('/tmp/axes_wheel_normal.mem',[-32768,32767,1234,-1,0,-23456],wheels=(98,-102))
golden('/tmp/axes_wheel_error.mem',[0]*6,True)
golden('/tmp/axes_wheel_new.mem',[1,-2,300,-400,5000,-6000],wheels=(-123,45))
golden('/tmp/axes_system_wheel.mem',[-32768,32767,1234,-1,0,-23456],wheels=(0,0))

golden('/tmp/axes_model_status.mem',[-32768,32767,1234,-1,0,-23456],header_override='M:81 F:0 N:1234 E:00')
golden('/tmp/axes_model_status_new.mem',[1,-2,300,-400,5000,-6000],header_override='M:81 F:0 N:2345 E:00')
