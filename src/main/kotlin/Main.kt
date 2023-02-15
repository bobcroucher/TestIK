import kotlin.math.*
import kotlin.math.IEEErem

const val shoulderLength = 37.0
const val elbowLength = 28.0

fun wrap(angle: Double) : Double {
    return angle.IEEErem(2.0 * Math.PI)
}

/** Converts joint angles to the end effector position.  */
fun forwardKinematics(inShoulder: Double, inElbow: Double) : Pair<Double,Double> {
    val shoulder = inShoulder + Math.PI / 2.0
    val elbow = inElbow - Math.PI / 2.0
    return Pair(
        shoulderLength * cos(shoulder) + elbowLength * cos(elbow),
        shoulderLength * sin(shoulder) + elbowLength * sin(elbow))
}

/** Converts the end effector position to joint angles. */
fun inverseKinematics(endPosition: Pair<Double, Double>) : Pair<Double, Double>{
    var relativePosition = endPosition

    // Flip when X is negative
    val isFlipped = relativePosition.first < 0.0
    if (isFlipped) {
        relativePosition = Pair(-relativePosition.first, relativePosition.second)
    }

    // Calculate angles
    var elbow = -acos((relativePosition.first.pow(2.0) + relativePosition.second.pow(2.0)
            - shoulderLength.pow(2.0) - elbowLength.pow(2.0)) / (2.0 * shoulderLength * elbowLength))

    if (elbow.isNaN()) {
        return Pair(0.0, 0.0)
    }

    var shoulder = Math.atan(relativePosition.second / relativePosition.first) -
            Math.atan((elbowLength * sin(elbow)) / (shoulderLength + elbowLength * cos(elbow)))

    // Invert shoulder angle if invalid
    val testPosition = forwardKinematics(shoulder, elbow)
    if (((testPosition.first-relativePosition.first).pow(2.0) +
        (testPosition.second-relativePosition.second).pow(2.0)).pow(0.5) > 1e-3) {
        shoulder += Math.PI
    }

    // Flip angles
    if (isFlipped) {
        shoulder = Math.PI - shoulder
        elbow = -elbow
    }

    // Wrap angles to correct ranges
    return Pair(wrap(shoulder + Math.PI/2.0), wrap(elbow + shoulder - Math.PI/2.0))
}

fun testIK() {
    var matches = 0
    var mismatches = 0
    for (shoulderDegrees in -175 .. 175 step(5)) {
        val shoulderRadians = Math.toRadians(shoulderDegrees.toDouble())
        for (elbowDegrees in -175..175 step(5)) {
            val elbowRadians = Math.toRadians(elbowDegrees.toDouble())
            val position = forwardKinematics(shoulderRadians, elbowRadians)
            val (shoulderIKRadians, elbowIKRadians) = inverseKinematics(position)
            val shoulderIKDegrees = Math.toDegrees(shoulderIKRadians)
            val elbowIKDegrees = Math.toDegrees(elbowIKRadians)
            if ((shoulderDegrees.toDouble()-shoulderIKDegrees).absoluteValue > 0.1 ||
                (elbowDegrees.toDouble()-elbowIKDegrees).absoluteValue > 0.1) {
                println("sh:$shoulderDegrees?=$shoulderIKDegrees  el:$elbowDegrees?=$elbowIKDegrees")
                mismatches++
            }
            else {
//                println("sh:$shoulderDegrees?=$shoulderIKDegrees  el:$elbowDegrees?=$elbowIKDegrees")
                matches++
            }
        }
    }
    assert(mismatches==0)
    println("matches=$matches mismatches=$mismatches")
}

fun main(args: Array<String>) {
    println("Hello World!")

    testIK()

    // Try adding program arguments via Run/Debug configuration.
    // Learn more about running applications: https://www.jetbrains.com/help/idea/running-applications.html.
    println("Program arguments: ${args.joinToString()}")
}