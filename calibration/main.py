from urllib.request import urlopen
from calibration import calibrate_ll

lower_bound = int(input('Lower bound of the images: '))
upper_bound = int(input('Upper bound of the images: '))

for i in range(lower_bound, upper_bound + 1):
    try:
        resource = urlopen(f'http://limelight.local:5801/snapshots/{i}.jpg')
    except:
        break

    output = open(f'{i}.jpg', 'wb')
    output.write(resource.read())
    output.close()
    print(f'Image {i} done.')

mtx, scaled_mtx, dist, fovx, fovy = calibrate_ll(9, 6, lower_bound, upper_bound)

print('\nMatrix (unnormalized):')
print(mtx)
print('Matrix:')
print(scaled_mtx)
print('Distortion coefficients:')
print(dist)
print("Horizontal FOV:", fovx)
print("Vertical FOV:", fovy)
