package org.firstinspires.ftc.teamcode

import kotlin.math.*

class PathCreator(val start: Point2D) {
    var points = mutableListOf<Point2D>(start)
    var segments = mutableListOf<LineSeg>()
    var completed = false
    var currentSeg: Int = 0
    var lookahead : Int = 5

    fun getTarget(current: Point2D) {
        var currentPoint = segments[currentSeg].getClosestPoint(current)/*
        if (segments[currentSeg].unitVec * lookahead > segments[currentSeg].maxPoint) {

        }*/
    }

    fun addPoint(point: Point2D) {
        if (!completed) {
            points.add(point)
            computeSegments()
        }
    }

    private fun computeSegments() {
        segments = mutableListOf<LineSeg>()
        for (i in 0 until points.size - 2) {
            segments.add(LineSeg(points[i], points[i+1]))
        }
    }
}

class Point2D(val x_val: Double, val y_val: Double) {
    val x = x_val
    val y = y_val
    
    operator fun plus(other: Point2D) = Point2D(x + other.x, y + other.y)
    operator fun minus(other: Point2D) = this + -other  // Does this work?

    operator fun times(other: Point2D) : Double = x * other.x + y * other.y  // Dot product
    operator fun times(other: Int) = Point2D(x * other, y * other)
    operator fun times(other: Double) = Point2D(x * other, y * other)

    operator fun div(other: Int) = Point2D(x / other, y / other)
    operator fun div(other: Double) = Point2D(x / other, y / other)

    override operator fun equals(other: Any?) : Boolean {
        if (other is Point2D) {
            return (x == other.x) and (y == other.y)
        }
        return false
    }

    fun distance(other: Point2D) : Double {
        return hypot(x - other.x, y - other.y)
    }

    operator fun unaryMinus() : Point2D {
        return Point2D(-x, -y)
    }
}

class LineSeg(val point1: Point2D, val point2: Point2D) {
    val maxPoint = if (point1.x > point2.x) point1 else point2
    val minPoint = if (point1.x <= point2.x) point1 else point2

    val length = point1.distance(point2)
    val unitVec = (point2 - point1) / length // Assuming line segment goes from point1 -> point2

    fun inRange(point: Point2D) : Boolean {
        return (point.x <= maxPoint.x) and (point.x >= minPoint.x)
    }

    fun getClosestPoint(point: Point2D) : Point2D {
        var V = maxPoint - minPoint
        var U = minPoint - point
        var t: Double = -(V * U) / (V * V)

        if (t > 1) {
            return maxPoint
        } else if(t < 0) {
            return minPoint
        } else {
            return maxPoint * (1 - t) + minPoint * t
        }
    }
}
