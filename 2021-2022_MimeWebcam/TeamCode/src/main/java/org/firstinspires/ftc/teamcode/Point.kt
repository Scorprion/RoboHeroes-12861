package org.firstinspires.ftc.teamcode

import kotlin.math.hypot
import kotlin.math.pow

data class Point(val x : Double, val y: Double) {
    fun dot(p: Point) : Double {
        return x * p.x + y * p.y
    }

    operator fun plus(p: Point) : Point {
        return Point(x + p.x, y + p.y)
    }

    operator fun minus(p: Point) : Point {
        return Point(x + p.x, y + p.y)
    }

    operator fun times(p: Point) : Point {
        return Point(x * p.x, y * p.y)
    }

    operator fun times(p: Double) : Point {
        return Point(x * p, y * p)
    }

    operator fun div(p: Point) : Point {
        return Point(x / p.x, y / p.y)
    }

    operator fun div(p: Double) : Point {
        return Point(x / p, y / p)
    }

    fun dist(p: Point) : Double {
        return hypot(x - p.x, y - p.y)
    }

    fun magnitude() : Double {
        return x.pow(2.0) + y.pow(2.0)
    }
}