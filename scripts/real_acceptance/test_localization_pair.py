from types import SimpleNamespace as O
from localization_probe import aligned_pair

def msg(sec, nanosec=0):return O(header=O(stamp=O(sec=sec,nanosec=nanosec)))
p1,p2=msg(10),msg(10,200000000)
s1=msg(10)
assert aligned_pair([p1,p2],[s1])==(p1,s1)
s2=msg(10,200000000)
assert aligned_pair([p1,p2],[s1,s2])==(p2,s2)
assert aligned_pair([p2],[msg(9,990000000)]) is None
assert aligned_pair([msg(11)],[msg(10,990000000)]) is not None
assert aligned_pair([p1],[]) is None
print('PASS: newest matched pair, late arrival, dropped frame, second rollover and empty input')
