from functools import partial

def convert(weights, frame):
    vals = weights[::2]
    w = weights[1::2]
    ret = 0
    for i in range(len(weights)/2):
        ret += frame[vals[i]] * w[1]
    return ret

frame = {
    'x' : 5,
    'y' : 6,
    'z' : 7
}

f = partial(convert,['x', 1, 'y', 2, 'z', 3])
print(f(frame))
