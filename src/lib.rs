/*! Call all functions with a Vector as argument the vector should contain:
 - the end point as origin
 - the end angle as angle in degrees in clockwise direction (eg. 0° facing north, 90° facing east, ...)
 - the circle radius as magnitude

Start Vector is in the origin facing in positive x-direction

Every struct defined here is 2 dimensional and uses f64 */

/// Point
pub struct Point {
    pub x: f64,
    pub y: f64,
}

/// Vector with origin, angle and magnitude
pub struct Vector {
    pub origin: Point,
    pub angle: f64,
    pub magnitude: f64,
}

/// Circle
pub struct Circle {
    pub center: Point,
    pub radius: f64,
}

/// Circle route with a circle and a angle for how long to drive on this circle
pub struct CircleRoute {
    pub circle: Circle,
    pub angle: f64,
}

/// Route with a start Circle, a tangent straight and a end Circle (eg. rsl, rsr, lsr, lsl)
pub struct RouteCSC {
    pub start: CircleRoute,
    pub tangent: Vector,
    pub end: CircleRoute,
}

/// Route with 3 Circles (eg. rlr, lrl) (not yet implemented)
#[allow(unused)]
pub struct RouteCCC {
    pub start: Circle,
    pub middle: Circle,
    pub end: Circle,
}

/// right straight right route
pub fn rsr(end: Vector) -> Result<RouteCSC, ()> {
    let mut route_csc = RouteCSC {
        start: CircleRoute {
            circle: Circle {
                center: Point {
                    x: end.magnitude,
                    y: 0.0,
                },
                radius: end.magnitude,
            },
            angle: 0.0,
        },
        tangent: Vector {
            origin: Point { x: 0.0, y: 0.0 },
            angle: 0.0,
            magnitude: 0.0,
        },
        end: CircleRoute {
            circle: Circle {
                center: Point { x: 0.0, y: 0.0 },
                radius: end.magnitude,
            },
            angle: 0.0,
        },
    };

    // get the center point by adding the end vector to the end point
    // this works because the argument is the angle in positive y direction
    // not positive x direction so we dont have to rotate it here anymore
    // the angle has to be counter clockwise though (thats why 360 - end.angle)
    route_csc.end.circle.center = Point {
        x: end.origin.x + end.magnitude * (360.0 - end.angle).to_radians().cos(),
        y: end.origin.y + end.magnitude * (360.0 - end.angle).to_radians().sin(),
    };

    // get the tangent pitch which is the same as the pitch between the two
    // circle centers since our circles have the same radius
    route_csc.tangent.angle = ((route_csc.end.circle.center.y - route_csc.start.circle.center.y)
        / (route_csc.end.circle.center.x - route_csc.start.circle.center.x))
        .atan()
        .to_degrees();

    // if the end circle center x value is smaller than the
    // start circle center x value
    // the angle would be 180° rotated so to prevent that:
    if route_csc.end.circle.center.x < route_csc.start.circle.center.x {
        route_csc.tangent.angle += 180.0;
    }

    // get the tangent magnitude this, again, is the same as the distance
    // between the two circle centers since our circles have the same radius
    route_csc.tangent.magnitude =
        ((route_csc.end.circle.center.x - route_csc.start.circle.center.x).powf(2.0)
            + (route_csc.end.circle.center.y - route_csc.start.circle.center.y).powf(2.0))
        .sqrt();

    // get the angle of the start circle
    route_csc.start.angle = 90.0 - route_csc.tangent.angle;

    // make the angle pretty
    if route_csc.start.angle < 0.0 {
        route_csc.start.angle += 360.0;
    }
    if route_csc.start.angle >= 360.0 {
        route_csc.start.angle -= 360.0;
    }

    // get the tangent origin by moving the vector from the start circle center
    // 90° to it's own direction and the magnitude of the circle radius
    route_csc.tangent.origin = Point {
        x: route_csc.start.circle.center.x
            + route_csc.start.circle.radius * (180.0 - route_csc.start.angle).to_radians().cos(),
        y: route_csc.start.circle.center.y
            + route_csc.start.circle.radius * (180.0 - route_csc.start.angle).to_radians().sin(),
    };

    // get the angle of the start circle
    // the angle where we start from the tangent equals the one we finish
    // so we can use that in here
    route_csc.end.angle = end.angle - route_csc.start.angle;

    // make the angle pretty
    if route_csc.end.angle < 0.0 {
        route_csc.end.angle += 360.0;
    }
    if route_csc.end.angle >= 360.0 {
        route_csc.end.angle -= 360.0;
    }

    Ok(route_csc)
}

/// left straight left route
pub fn lsl(end: Vector) -> Result<RouteCSC, ()> {
    let mut route_csc = RouteCSC {
        start: CircleRoute {
            circle: Circle {
                center: Point {
                    x: -end.magnitude,
                    y: 0.0,
                },
                radius: end.magnitude,
            },
            angle: 0.0,
        },
        tangent: Vector {
            origin: Point { x: 0.0, y: 0.0 },
            angle: 0.0,
            magnitude: 0.0,
        },
        end: CircleRoute {
            circle: Circle {
                center: Point { x: 0.0, y: 0.0 },
                radius: end.magnitude,
            },
            angle: 0.0,
        },
    };

    // get the center point by adding the end vector to the end point
    // we have to rotate the vector 180° (90° because the given angle is from the y axis
    // and 90 more to not get the tangent but the vector to the center point)
    // and again we have to use the counter clockwise direction
    route_csc.end.circle.center = Point {
        x: end.origin.x + end.magnitude * (180.0 - end.angle).to_radians().cos(),
        y: end.origin.y + end.magnitude * (180.0 - end.angle).to_radians().sin(),
    };

    // get the tangent pitch which is the same as the pitch between the two
    // circle centers since our circles have the same radius
    route_csc.tangent.angle = ((route_csc.end.circle.center.y - route_csc.start.circle.center.y)
        / (route_csc.end.circle.center.x - route_csc.start.circle.center.x))
        .atan()
        .to_degrees();

    // if the end circle center x value is smaller than the
    // start circle center x value
    // the angle would be 180° rotated so to prevent that:
    if route_csc.end.circle.center.x < route_csc.start.circle.center.x {
        route_csc.tangent.angle += 180.0;
    }

    // make the angle positive
    if route_csc.tangent.angle < 0.0 {
        route_csc.tangent.angle += 360.0;
    }

    // get the tangent magnitude this, again, is the same as the distance
    // between the two circle centers since our circles have the same radius
    route_csc.tangent.magnitude = ((route_csc.end.circle.center.x
        - route_csc.start.circle.center.x)
        .abs()
        .powf(2.0)
        + (route_csc.end.circle.center.y - route_csc.start.circle.center.y)
            .abs()
            .powf(2.0))
    .sqrt();

    // get the angle of the start circle
    route_csc.start.angle = route_csc.tangent.angle - 90.0;

    // make the angle pretty
    if route_csc.start.angle < 0.0 {
        route_csc.start.angle += 360.0;
    }
    if route_csc.start.angle >= 360.0 {
        route_csc.start.angle -= 360.0;
    }

    // get the tangent origin by moving the vector from the start circle center
    // 90° to it's own direction and the magnitude of the circle radius
    route_csc.tangent.origin = Point {
        x: route_csc.start.circle.center.x
            + route_csc.start.circle.radius * route_csc.start.angle.to_radians().cos(),
        y: route_csc.start.circle.center.y
            + route_csc.start.circle.radius * route_csc.start.angle.to_radians().sin(),
    };

    // get the angle of the start circle
    // the angle where we start from the tangent equals the one we finish
    // so we can use that in here
    route_csc.end.angle = end.angle - route_csc.start.angle;

    // make the angle pretty
    if route_csc.end.angle < 0.0 {
        route_csc.end.angle += 360.0;
    }
    if route_csc.end.angle >= 360.0 {
        route_csc.end.angle -= 360.0;
    }

    Ok(route_csc)
}

/// right straight left route
pub fn rsl(end: Vector) -> Result<RouteCSC, ()> {
    let mut route_csc = RouteCSC {
        start: CircleRoute {
            circle: Circle {
                center: Point {
                    x: end.magnitude,
                    y: 0.0,
                },
                radius: end.magnitude,
            },
            angle: 0.0,
        },
        tangent: Vector {
            origin: Point { x: 0.0, y: 0.0 },
            angle: 0.0,
            magnitude: 0.0,
        },
        end: CircleRoute {
            circle: Circle {
                center: Point { x: 0.0, y: 0.0 },
                radius: end.magnitude,
            },
            angle: 0.0,
        },
    };

    // get the center point by adding the end vector to the end point
    // we have to rotate the vector 180° (90° because the given angle is from the y axis
    // and 90 more to not get the tangent but the vector to the center point)
    // and again we have to use the counter clockwise direction
    route_csc.end.circle.center = Point {
        x: end.origin.x + end.magnitude * (180.0 - end.angle).to_radians().cos(),
        y: end.origin.y + end.magnitude * (180.0 - end.angle).to_radians().sin(),
    };

    // check if inside tangent can even be constructed
    if ((route_csc.end.circle.center.x - route_csc.start.circle.center.x).powf(2.0)
        + (route_csc.end.circle.center.y - route_csc.start.circle.center.y).powf(2.0))
    .sqrt()
        < 2.0 * end.magnitude
    {
        return Err(());
    }

    // get the tangent length via some simple trigonometry
    route_csc.tangent.magnitude =
        ((route_csc.end.circle.center.x - route_csc.start.circle.center.x).powf(2.0)
            + (route_csc.end.circle.center.y - route_csc.start.circle.center.y).powf(2.0)
            - (2.0 * end.magnitude).powf(2.0))
        .sqrt();

    // tangent middle is the same as the middle of the straight from the center of the start
    let tangent_middle = Point {
        x: (route_csc.end.circle.center.x + route_csc.start.circle.center.x) / 2.0,
        y: (route_csc.end.circle.center.y + route_csc.start.circle.center.y) / 2.0,
    };

    // get the tangent angle
    route_csc.tangent.angle = ((route_csc.end.circle.center.y - tangent_middle.y)
        / (route_csc.end.circle.center.x - tangent_middle.x))
        .atan()
        .to_degrees()
        - (2.0 * end.magnitude / route_csc.tangent.magnitude)
            .atan()
            .to_degrees();

    // if the end circle center x value is smaller than the
    // start circle center x value
    // the angle would be 180° rotated so to prevent that:
    if route_csc.end.circle.center.x < route_csc.start.circle.center.x {
        route_csc.tangent.angle += 180.0;
    }

    // get the angle of the start circle
    route_csc.start.angle = 90.0 - route_csc.tangent.angle;

    // make the angle pretty
    if route_csc.start.angle < 0.0 {
        route_csc.start.angle += 360.0;
    }
    if route_csc.start.angle >= 360.0 {
        route_csc.start.angle -= 360.0;
    }

    // get the tangent origin by moving the vector from the start circle center
    // along its right angle vector
    route_csc.tangent.origin = Point {
        x: route_csc.start.circle.center.x
            + route_csc.start.circle.radius * (180.0 - route_csc.start.angle).to_radians().cos(),
        y: route_csc.start.circle.center.y
            + route_csc.start.circle.radius * (180.0 - route_csc.start.angle).to_radians().sin(),
    };

    // get the angle of the end circle
    route_csc.end.angle = (90.0 - end.angle) - route_csc.tangent.angle;

    // make the angle pretty
    if route_csc.end.angle < 0.0 {
        route_csc.end.angle += 360.0;
    }
    if route_csc.end.angle >= 360.0 {
        route_csc.end.angle -= 360.0;
    }

    Ok(route_csc)
}

/// left straight right route
pub fn lsr(end: Vector) -> Result<RouteCSC, ()> {
    let mut route_csc = RouteCSC {
        start: CircleRoute {
            circle: Circle {
                center: Point {
                    x: -end.magnitude,
                    y: 0.0,
                },
                radius: end.magnitude,
            },
            angle: 0.0,
        },
        tangent: Vector {
            origin: Point { x: 0.0, y: 0.0 },
            angle: 0.0,
            magnitude: 0.0,
        },
        end: CircleRoute {
            circle: Circle {
                center: Point { x: 0.0, y: 0.0 },
                radius: end.magnitude,
            },
            angle: 0.0,
        },
    };

    // get the center point by adding the end vector to the end point
    // this works because the argument is the angle in positive y direction
    // not positive x direction so we dont have to rotate it here anymore
    // the angle has to be counter clockwise though (thats why 360 - end.angle)
    route_csc.end.circle.center = Point {
        x: end.origin.x + end.magnitude * (360.0 - end.angle).to_radians().cos(),
        y: end.origin.y + end.magnitude * (360.0 - end.angle).to_radians().sin(),
    };

    // check if inside tangent can even be constructed
    if ((route_csc.end.circle.center.x - route_csc.start.circle.center.x).powf(2.0)
        + (route_csc.end.circle.center.y - route_csc.start.circle.center.y).powf(2.0))
    .sqrt()
        < 2.0 * end.magnitude
    {
        return Err(());
    }

    // get the tangent length via some simple trigonometry
    route_csc.tangent.magnitude =
        ((route_csc.end.circle.center.x - route_csc.start.circle.center.x).powf(2.0)
            + (route_csc.end.circle.center.y - route_csc.start.circle.center.y).powf(2.0)
            - (2.0 * end.magnitude).powf(2.0))
        .sqrt();

    // tangent middle is the same as the middle of the straight from the center of the start
    let tangent_middle = Point {
        x: (route_csc.end.circle.center.x + route_csc.start.circle.center.x) / 2.0,
        y: (route_csc.end.circle.center.y + route_csc.start.circle.center.y) / 2.0,
    };

    // get the tangent angle
    route_csc.tangent.angle = ((route_csc.end.circle.center.y - tangent_middle.y)
        / (route_csc.end.circle.center.x - tangent_middle.x))
        .atan()
        .to_degrees()
        + (2.0 * end.magnitude / route_csc.tangent.magnitude)
            .atan()
            .to_degrees();

    // if the end circle center x value is smaller than the
    // start circle center x value
    // the angle would be 180° rotated so to prevent that:
    if route_csc.end.circle.center.x < route_csc.start.circle.center.x {
        route_csc.tangent.angle += 180.0;
    }

    // get the angle of the start circle
    route_csc.start.angle = route_csc.tangent.angle - 90.0;

    // make the angle pretty
    if route_csc.start.angle < 0.0 {
        route_csc.start.angle += 360.0;
    }
    if route_csc.start.angle >= 360.0 {
        route_csc.start.angle -= 360.0;
    }

    // get the tangent origin by moving the vector from the start circle center
    // 90° to it's own direction and the magnitude of the circle radius
    route_csc.tangent.origin = Point {
        x: route_csc.start.circle.center.x
            + route_csc.start.circle.radius * route_csc.start.angle.to_radians().cos(),
        y: route_csc.start.circle.center.y
            + route_csc.start.circle.radius * route_csc.start.angle.to_radians().sin(),
    };

    // get the angle of the end circle
    route_csc.end.angle = (90.0 - end.angle) - route_csc.tangent.angle;

    // make the angle pretty
    if route_csc.end.angle < 0.0 {
        route_csc.end.angle += 360.0;
    }
    if route_csc.end.angle >= 360.0 {
        route_csc.end.angle -= 360.0;
    }

    Ok(route_csc)
}
