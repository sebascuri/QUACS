from math import cos, sin, asin, atan2, acos

R = 6371000
def gps2cart(*args, **kwargs):
	if len(args) == 2:
		alt = 0
		lat = args[0]
		lon = args[1]
	elif len(args) == 3:
		alt = args[0]
		lat = args[1]
		lon = args[2]
	elif len(args) == 1:
		return gps2cart(**args[0])

	for key, value in kwargs.items():
		if key == 'altitude':
			alt = value
		elif key == 'longitude':
			lon = value
		elif key == 'latitude':
			lat = value

	r = R + alt
	x = r * cos(lat) * cos(lon)
	y = r * cos(lat) * sin(lon)
	z = r * sin(lat) 

	return dict(x = x, y = y, z = z)

def cart2gps(*args, **kwargs):
	if len(args) == 2:
		z = 0
		x = args[0]
		y = args[1]
	elif len(args) == 3:
		x = args[0]
		y = args[1]
		z = args[2]
	elif len(args) == 1:
		return cart2gps(**args[0])

	for key, value in kwargs.items():
		if key == 'x':
			x = value
		elif key == 'y':
			y = value
		elif key == 'z':
			z = value


	r = (x**2 + y**2 + z**2)**0.5 + R 

	alt = r - R 
	lat = asin( z / r )
	lon = 2 * atan2(y, x + (x**2 + y**2)**0.5 )

	return dict(altitude = alt, latitude = lat, longitude = lon)

def main():
	from math import pi
	position = gps2cart(altitude = 10, latitude = pi/2, longitude = pi/4)

	print position
	gps = cart2gps( **position)
	print gps

if __name__ == '__main__': main()