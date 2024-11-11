from gpiozero import DistanceSensor

ultrasonic = DistanceSensor(echo=17, trigger=4)

print("Distance: " + ultrasonic.distance)