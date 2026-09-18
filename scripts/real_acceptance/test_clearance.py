from clearance import first_contact
b=[-.3,.3,-.2,.2]
assert abs(first_contact([(.55,0)],b,(1,0))-.15)<1e-9
assert first_contact([(0,.55)],b,(1,0)) is None  # side wall is outside forward swept strip
assert abs(first_contact([(0,.55)],b,(0,1))-.25)<1e-9
assert first_contact([(.2,0)],b,(1,0))==0
assert abs(first_contact([(-.6,0)],b,(-1,0))-.2)<1e-9
assert first_contact([],b,(1,0)) is None
print('PASS: forward/side/backward contact, overlap and unknown space')

from clearance import circle_contact
assert abs(circle_contact([(.55,0)],(1,0))-.25)<1e-9
assert circle_contact([(0,.55)],(1,0)) is None
assert circle_contact([(.1,0)],(1,0))==0
assert circle_contact([(-.55,0)],(1,0)) is None
assert abs(circle_contact([(-.55,0)],(-1,0))-.25)<1e-9
print('PASS: user-confirmed radius .20 m swept-circle contacts')
