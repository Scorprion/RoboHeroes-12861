package org.firstinspires.ftc.teamcode

import org.apache.commons.math3.analysis.interpolation.AkimaSplineInterpolator
import org.apache.commons.math3.analysis.interpolation.LinearInterpolator
import kotlin.math.*


class SmoothPath {
    constructor(x: DoubleArray, y: DoubleArray) { // Constructor
        var points: Array<DoubleArray> = arrayOf(x, y)
        var interpolator: AkimaSplineInterpolator = AkimaSplineInterpolator()
        interpolator.interpolate(x, y)
    }

    constructor(x: DoubleArray, y: DoubleArray, nBetween: Int) { // Constructor
        var points = x.zip(y).asSequence()

        for ((point1, point2) in points.windowed(2, 1, false)) {
            val slope = (point2.second - point1.second) / (point2.first - point1.first)
        }


        var interpolator: AkimaSplineInterpolator = AkimaSplineInterpolator()
        interpolator.interpolate(x, y)
    }

    fun getTarget(x: Double, y: Double) {
        // Get closest point

    }
}

class StraightPath {
    private var magnitude: Double = 0.0
    private var unitVec: List<Double> = emptyList()
    private var steps: Int = 0
    private var points: List<List<Double>> = emptyList()

    /*
    constructor(start: DoubleArray, end: DoubleArray, nPoints: Int) {
        val slope = (end[1] - start[1]) / (end[0] - start[0])
        val step = (end[0] - start[0]) / nPoints
        val points = DoubleArray(nPoints) { it + step }
                .map { x -> slope * (x - start[0])   + start[1] }
    }*/

    constructor(start: DoubleArray, end: DoubleArray, spacing: Double) {
        this.magnitude = hypot(start[0] - end[0], start[1] - end[1])
        this.unitVec = arrayOf(end[0] - start[0], end[1] - start[1]).map {d -> d / magnitude}
        this.steps = ceil(magnitude / spacing).toInt()
        this.points = List(steps) { idx -> unitVec.map {d -> d * idx} }
    }

    fun getClosest(point: DoubleArray): List<Double>? {
        return points.minWithOrNull( Comparator.comparingDouble { hypot(it[0] - point[0], it[1] - point[1]) } )
    }

}