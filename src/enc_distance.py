from math import pi

encoder_ticks = 206.25
wheel_dia = 6.7

def encToDist(ticks):
    circum = wheel_dia*pi
    dist_per_rev = circum/100

    distance = (ticks/encoder_ticks)*dist_per_rev

    return(distance)

ticks = 826
distance = encToDist(ticks)
print(distance)