"""Camera-facing timing text using IsaacGym's viewer line API."""
import numpy as np

# Five by seven pixels; horizontal strokes keep text visible on a grey scene.
FONT = {
    '0': '01110/10001/10011/10101/11001/10001/01110',
    '1': '00100/01100/00100/00100/00100/00100/01110',
    '2': '01110/10001/00001/00010/00100/01000/11111',
    '3': '11110/00001/00001/01110/00001/00001/11110',
    '4': '00010/00110/01010/10010/11111/00010/00010',
    '5': '11111/10000/10000/11110/00001/00001/11110',
    '6': '01110/10000/10000/11110/10001/10001/01110',
    '7': '11111/00001/00010/00100/01000/01000/01000',
    '8': '01110/10001/10001/01110/10001/10001/01110',
    '9': '01110/10001/10001/01111/00001/00001/01110',
    'A': '01110/10001/10001/11111/10001/10001/10001',
    'B': '11110/10001/10001/11110/10001/10001/11110',
    'C': '01110/10001/10000/10000/10000/10001/01110',
    'D': '11110/10001/10001/10001/10001/10001/11110',
    'E': '11111/10000/10000/11110/10000/10000/11111',
    'G': '01110/10001/10000/10111/10001/10001/01110',
    'H': '10001/10001/10001/11111/10001/10001/10001',
    'I': '11111/00100/00100/00100/00100/00100/11111',
    'J': '00111/00010/00010/00010/00010/10010/01100',
    'K': '10001/10010/10100/11000/10100/10010/10001',
    'L': '10000/10000/10000/10000/10000/10000/11111',
    'M': '10001/11011/10101/10101/10001/10001/10001',
    'N': '10001/11001/11001/10101/10011/10011/10001',
    'O': '01110/10001/10001/10001/10001/10001/01110',
    'P': '11110/10001/10001/11110/10000/10000/10000',
    'R': '11110/10001/10001/11110/10100/10010/10001',
    'S': '01111/10000/10000/01110/00001/00001/11110',
    'T': '11111/00100/00100/00100/00100/00100/00100',
    'U': '10001/10001/10001/10001/10001/10001/01110',
    'V': '10001/10001/10001/10001/10001/01010/00100',
    'W': '10001/10001/10001/10101/10101/10101/01010',
    'X': '10001/10001/01010/00100/01010/10001/10001',
    ':': '00000/00100/00100/00000/00100/00100/00000',
    '.': '00000/00000/00000/00000/00000/00100/00100',
    '-': '00000/00000/00000/11111/00000/00000/00000',
}


def glyph_lines(pattern):
    lines=[]
    for y,bits in enumerate(pattern.split('/')):
        for x,bit in enumerate(bits):
            if bit=='1':
                for stroke in (0., .3, .6):
                    lines.append([[x,y+stroke],[x+.85,y+stroke]])
    return np.asarray(lines,dtype=np.float32)


GLYPHS={char:glyph_lines(pattern) for char,pattern in FONT.items()}


def draw_time_hud(gym, viewer, camera, target, rows):
    camera, target = np.asarray(camera), np.asarray(target)
    forward = target-camera
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0., 0., 1.])
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    anchor = camera + .50*forward - .31*right + .165*up
    pixel = .0020
    vertices, colors = [], []
    for row_index, text in enumerate(rows):
        color = (1., .95, .25) if row_index == 3 else (.25, 1., 1.)
        for column, char in enumerate(text):
            if char not in GLYPHS:
                continue
            xy=GLYPHS[char]+np.asarray([column*6,row_index*10])
            vertices.append(anchor+xy[:,:,[0]]*pixel*right-xy[:,:,[1]]*pixel*up)
            colors.append(np.tile(color,(len(xy),1)))
    vertices=np.concatenate(vertices).astype(np.float32)
    colors=np.concatenate(colors).astype(np.float32)
    gym.clear_lines(viewer)
    gym.add_lines(viewer, None, len(vertices), vertices, colors)
