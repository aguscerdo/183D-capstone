
desired_h= 5
actual_h = 350
difference = 180 - abs(abs(desired_h - actual_h) - 180)
print(difference)

if(actual_h < desired_h ):
    if desired_h - actual_h > 180:
        print("go left")
    else:
        print("go right")
else:
    if actual_h - desired_h > 180:
        print("go right")
    else:
        print("go left")