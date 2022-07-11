import re
import numpy as np
f = open("../hessian.txt", 'r')

_RE_COMBINE_WHITESPACE = re.compile(r"\s+")

matrix = []
for l in f.readlines():
    line = _RE_COMBINE_WHITESPACE.sub(" ", l).strip()
    line = line.split(' ')
    matrix.append(line)
    # break

hessian = np.array(matrix, dtype=np.float64)
print(hessian.shape)

def is_pos_def(x):
    return np.all(np.linalg.eigvals(x) > 0)

rank = np.linalg.matrix_rank(hessian)

print("Rank: {}".format(rank))
print("Is pos def: {}".format(is_pos_def(hessian)))
