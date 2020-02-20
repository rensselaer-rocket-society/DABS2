from math import log as ln

g = 9.807

def v2e(v):
    return 0.5*v*v

def dz(k,e):
    return ln(1+k*e/g)/k

def dzdk(k,e,dz0=None):
    if dz0 is None:
        dz0 = dz(k,e)
    return ( e/(g+e*k) - dz0 )/k

# def kfordz(e,dz0,k0):
#     k = k0
#     currZ = dz(k,e)
#     i = 1
#     while abs(currZ-dz0)>0.01:
#         k2 = k - (currZ-dz0)/dzdk(k,e,currZ)
#         if k2 < 0:
#             print("OVERSHOOT RECOVERY!")
#             k2 = k/2 # Protection for leaving reasonable domain (if value to high)
#         k = k2
#         currZ = dz(k,e)
#         print("Iteration {}:\tk={:f}\tdz={:f}\t({:F})".format(i,k,currZ,dz0))
#         i = i+1
#     print()
#     return k

def kfordz2(e,dz0,k0):
    k = k0
    currZ = dz(k,e)
    i = 1
    while abs(currZ-dz0)>0.1:
        k2 = k*(1+(currZ-dz0)/(currZ - 1/(g/e + k)))
        if k2 < 0:
            print("OVERSHOOT RECOVERY!")
            k2 = k/2 # Protection for leaving reasonable domain (if value to high)
        k = k2
        currZ = dz(k,e)
        print("Iteration {}:\tk={:f}\tdz={:f}\t({:F})".format(i,k,currZ,dz0))
        i = i+1
    print("Fractional Error={}".format((currZ-dz0)/dz0))
    print()
    return k

k = 0.002
for i in range(0,100,10):
    k = kfordz2(v2e(300-i),300-i,k)