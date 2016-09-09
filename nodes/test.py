from naoqi import ALProxy
import time


# Choregraphe simplified export in Python.
names = list()
times = list()
keys = list()

names.append("HeadPitch")
times.append([0.96, 1.64])
keys.append([0.25767, 0.352778])

names.append("HeadYaw")
times.append([0.96, 1.64])
keys.append([-0.177985, -0.105888])

names.append("LAnklePitch")
times.append([1.52])
keys.append([-0.265424])

names.append("LAnkleRoll")
times.append([1.52])
keys.append([-0.11961])

names.append("LElbowRoll")
times.append([0.88, 1.56])
keys.append([-0.710201, -0.513848])

names.append("LElbowYaw")
times.append([0.88, 1.56])
keys.append([-1.66903, -1.76107])

names.append("LHand")
times.append([0.88, 1.56])
keys.append([0.358182, 0.360025])

names.append("LHipPitch")
times.append([1.52])
keys.append([-0.711735])

names.append("LHipRoll")
times.append([1.52])
keys.append([0.127364])

names.append("LHipYawPitch")
times.append([1.52])
keys.append([-0.427944])

names.append("LKneePitch")
times.append([1.52])
keys.append([1.07836])

names.append("LShoulderPitch")
times.append([0.88, 1.56])
keys.append([1.30232, 1.38209])

names.append("LShoulderRoll")
times.append([0.88, 1.56])
keys.append([0.300622, 0.289883])

names.append("LWristYaw")
times.append([0.88, 1.56])
keys.append([-0.263545, -0.273093])

names.append("RAnklePitch")
times.append([1.52])
keys.append([-0.337438])

names.append("RAnkleRoll")
times.append([1.52])
keys.append([0.0445279])

names.append("RElbowRoll")
times.append([0.8, 1.48, 2.5])
keys.append([1.18682, 0.139636, 0.0733])

names.append("RElbowYaw")
times.append([0.8, 1.48, 2.5])
keys.append([0.0459781, -0.047596, 1.1711])

names.append("RHand")
times.append([0.8, 1.48])
keys.append([0.845478, 0.8])

names.append("RHipPitch")
times.append([1.52])
keys.append([-0.628982])

names.append("RHipRoll")
times.append([1.52])
keys.append([0.016916])

names.append("RKneePitch")
times.append([1.52])
keys.append([1.07077])

names.append("RShoulderPitch")
times.append([0.8, 1.48, 2.5])
keys.append([0.83914, 0.909704, 0.5498])

names.append("RShoulderRoll")
times.append([0.8, 1.48, 2.5])
keys.append([-0.139626, -0.012314, 0.3107])

names.append("RWristYaw")
times.append([0.8, 1.48, 2.5])
keys.append([0.488692, 0.477032, 1.2758])









# Choregraphe simplified export in Python.
names2 = list()
times2 = list()
keys2 = list()

names2.append("HeadPitch")
times2.append([1, 1.72, 2.72])
keys2.append([-0.325251, -0.177985, -0.0349066])

names2.append("HeadYaw")
times2.append([1, 1.72])
keys2.append([-0.0890139, -0.092082])

names2.append("LAnklePitch")
times2.append([1.6, 2.6])
keys2.append([0.102736, 0.151824])

names2.append("LAnkleRoll")
times2.append([1.6, 2.6])
keys2.append([-0.087396, -0.06592])

names2.append("LElbowRoll")
times2.append([0.92, 1.64, 2.64])
keys2.append([-1.5447, -1.55697, -1.54163])

names2.append("LElbowYaw")
times2.append([0.92, 1.64, 2.64])
keys2.append([-1.01862, -0.144238, -0.138102])

names.append("LHand")
times.append([1.64, 2.64])
keys.append([0.214545, 0.216389])

names2.append("LHipPitch")
times2.append([1.6, 2.6])
keys2.append([0.325251, 0.22554])

names2.append("LHipRoll")
times2.append([1.6, 2.6])
keys2.append([0.121228, 0.0767419])

names2.append("LHipYawPitch")
times2.append([1.6, 2.6])
keys2.append([-0.398797, -0.450955])

names2.append("LKneePitch")
times2.append([1.6, 2.6])
keys2.append([-0.0923461, -0.0923461])

names2.append("LShoulderPitch")
times2.append([0.92, 1.64, 2.64])
keys2.append([0.538392, 0.18864, 0.223922])

names2.append("LShoulderRoll")
times2.append([0.92, 1.64, 2.64])
keys2.append([0.0275701, 0.00869999, 0.016832])

names2.append("LWristYaw")
times2.append([1.64, 2.64])
keys2.append([-0.20944, -0.196393])

names2.append("RAnklePitch")
times2.append([1.6, 2.6])
keys2.append([0.0598679, 0.139636])

names2.append("RAnkleRoll")
times2.append([1.6, 2.6])
keys2.append([0.075208, 0.06447])

names2.append("RElbowRoll")
times2.append([0.84, 1.56, 2.56])
keys2.append([1.16281, 1.31008, 1.28707])

names2.append("RElbowYaw")
times2.append([0.84, 1.56, 2.56])
keys2.append([0.753151, 0.168698, 0.167164])

names2.append("RHand")
times2.append([1.56, 2.56])
keys2.append([0.221818, 0.224753])

names2.append("RHipPitch")
times2.append([1.6, 2.6])
keys2.append([0.377323, 0.240796])

names2.append("RHipRoll")
times2.append([1.6, 2.6])
keys2.append([-0.115008, -0.084328])

names2.append("RKneePitch")
times2.append([1.6, 2.6])
keys2.append([-0.103083, -0.102736])

names2.append("RShoulderPitch")
times2.append([0.84, 1.56, 2.56])
keys2.append([1.18582, 0.79312, 0.83147])

names2.append("RShoulderRoll")
times2.append([0.84, 1.56, 2.56])
keys2.append([-0.012314, -0.013848, -0.0261199])

names2.append("RWristYaw")
times2.append([1.56, 2.56])
keys2.append([-0.0174533, -0.029188])


if __name__ == '__main__':

	motionProxy = ALProxy("ALMotion", "10.0.0.10", 9559)
	postureProxy = ALProxy("ALRobotPosture", "10.0.0.10", 9559)


	motionProxy.post.angleInterpolationBezier(names2, times2, keys2)
	motionProxy.post.angleInterpolationBezier(names, times, keys)

	time.sleep(2)
	postureProxy.goToPosture("Stand", 0.5)

	time.sleep(2)
	postureProxy.goToPosture("Stand", 0.5)