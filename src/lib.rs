//! Call all functions with a Vector as argument the vector should contain:
//!  - the end point as origin
//!  - the end angle as angle in degrees in clockwise direction (eg. 0° facing north, 90° facing east, ...)
//!  - the circle radius as magnitude
//!
//! Start Vector is in the origin facing in positive x-direction
//!
//! Every struct defined here is 2 dimensional and uses f64

use euclid::{Point2D, Rotation2D, UnknownUnit};

pub type Angle = euclid::Angle<f64>;
pub type Point = Point2D<f64, UnknownUnit>;
type Vector2D = euclid::Vector2D<f64, UnknownUnit>;
type Rotation = Rotation2D<f64, UnknownUnit, UnknownUnit>;

/// Vector with origin, angle and magnitude
pub struct Vector {
    pub origin: Point,
    pub angle: Angle,
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
    pub angle: Angle,
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
                center: Point::new(end.magnitude, 0.0),
                radius: end.magnitude,
            },
            angle: Angle::zero(),
        },
        tangent: Vector {
            origin: Point::zero(),
            angle: Angle::zero(),
            magnitude: 0.0,
        },
        end: CircleRoute {
            circle: Circle {
                center: Point::zero(),
                radius: end.magnitude,
            },
            angle: Angle::zero(),
        },
    };

    // get the center point by adding the end vector to the end point
    // this works because the argument is the angle in positive y direction
    // not positive x direction so we dont have to rotate it here anymore
    // the angle has to be counter clockwise though (thats why we use the inverse end.angle)
    route_csc.end.circle.center = end.origin
        + Rotation::new(end.angle)
            .inverse()
            .transform_vector(Vector2D::new(end.magnitude, 0.0));

    // get the tangent pitch which is the same as the pitch between the two
    // circle centers since our circles have the same radius
    route_csc.tangent.angle = Angle::radians(
        ((route_csc.end.circle.center.y - route_csc.start.circle.center.y)
            / (route_csc.end.circle.center.x - route_csc.start.circle.center.x))
            .atan(),
    );

    // if the end circle center x value is smaller than the
    // start circle center x value
    // the angle would be 180° rotated so to prevent that:
    if route_csc.end.circle.center.x < route_csc.start.circle.center.x {
        route_csc.tangent.angle += Angle::pi();
    }

    // get the tangent magnitude this, again, is the same as the distance
    // between the two circle centers since our circles have the same radius
    route_csc.tangent.magnitude =
        ((route_csc.end.circle.center.x - route_csc.start.circle.center.x).powi(2)
            + (route_csc.end.circle.center.y - route_csc.start.circle.center.y).powi(2))
        .sqrt();

    // get the angle of the start circle
    route_csc.start.angle = (Angle::frac_pi_2() - route_csc.tangent.angle).positive();

    // get the tangent origin by moving the vector from the start circle center
    // 90° to it's own direction and the magnitude of the circle radius
    route_csc.tangent.origin = route_csc.start.circle.center
        + Rotation::new(Angle::pi() - end.angle)
            .transform_vector(Vector2D::new(route_csc.start.circle.radius, 0.0));

    // get the angle of the start circle
    // the angle where we start from the tangent equals the one we finish
    // so we can use that in here
    route_csc.end.angle = (end.angle - route_csc.start.angle).positive();

    Ok(route_csc)
}

/// left straight left route
pub fn lsl(end: Vector) -> Result<RouteCSC, ()> {
    let mut route_csc = RouteCSC {
        start: CircleRoute {
            circle: Circle {
                center: Point::new(-end.magnitude, 0.0),
                radius: end.magnitude,
            },
            angle: Angle::zero(),
        },
        tangent: Vector {
            origin: Point::zero(),
            angle: Angle::zero(),
            magnitude: 0.0,
        },
        end: CircleRoute {
            circle: Circle {
                center: Point::zero(),
                radius: end.magnitude,
            },
            angle: Angle::zero(),
        },
    };

    // get the center point by adding the end vector to the end point
    // we have to rotate the vector π (π/2 because the given angle is from the y axis
    // and π/2 more to not get the tangent but the vector to the center point)
    // and again we have to use the counter clockwise direction
    route_csc.end.circle.center = end.origin
        + Rotation::new(Angle::pi() - end.angle)
            .transform_vector(Vector2D::new(end.magnitude, 0.0));

    // get the tangent pitch which is the same as the pitch between the two
    // circle centers since our circles have the same radius
    route_csc.tangent.angle = Angle::radians(
        ((route_csc.end.circle.center.y - route_csc.start.circle.center.y)
            / (route_csc.end.circle.center.x - route_csc.start.circle.center.x))
            .atan(),
    )
    .positive();

    // if the end circle center x value is smaller than the
    // start circle center x value
    // the angle would be π rotated so to prevent that:
    if route_csc.end.circle.center.x < route_csc.start.circle.center.x {
        route_csc.tangent.angle = (route_csc.tangent.angle + Angle::pi()).positive();
    }

    // get the tangent magnitude this, again, is the same as the distance
    // between the two circle centers since our circles have the same radius
    route_csc.tangent.magnitude = ((route_csc.end.circle.center.x
        - route_csc.start.circle.center.x)
        .abs()
        .powi(2)
        + (route_csc.end.circle.center.y - route_csc.start.circle.center.y)
            .abs()
            .powi(2))
    .sqrt();

    // get the angle of the start circle
    route_csc.start.angle = (route_csc.tangent.angle - Angle::frac_pi_2()).positive();

    // get the tangent origin by moving the vector from the start circle center
    // 90° to it's own direction and the magnitude of the circle radius
    route_csc.tangent.origin = route_csc.start.circle.center
        + Rotation::new(route_csc.start.angle)
            .transform_vector(Vector2D::new(route_csc.start.circle.radius, 0.0));

    // get the angle of the start circle
    // the angle where we start from the tangent equals the one we finish
    // so we can use that in here
    route_csc.end.angle = (end.angle - route_csc.start.angle).positive();

    Ok(route_csc)
}

/// right straight left route
pub fn rsl(end: Vector) -> Result<RouteCSC, ()> {
    let mut route_csc = RouteCSC {
        start: CircleRoute {
            circle: Circle {
                center: Point::new(end.magnitude, 0.0),
                radius: end.magnitude,
            },
            angle: Angle::zero(),
        },
        tangent: Vector {
            origin: Point::zero(),
            angle: Angle::zero(),
            magnitude: 0.0,
        },
        end: CircleRoute {
            circle: Circle {
                center: Point::zero(),
                radius: end.magnitude,
            },
            angle: Angle::zero(),
        },
    };

    // get the center point by adding the end vector to the end point
    // we have to rotate the vector π (π/2 because the given angle is from the y axis
    // and π/2 more to not get the tangent but the vector to the center point)
    // and again we have to use the counter clockwise direction
    route_csc.end.circle.center = end.origin
        + Rotation::new(Angle::pi() - end.angle)
            .transform_vector(Vector2D::new(end.magnitude, 0.0));

    // check if inside tangent can even be constructed
    if ((route_csc.end.circle.center.x - route_csc.start.circle.center.x).powi(2)
        + (route_csc.end.circle.center.y - route_csc.start.circle.center.y).powi(2))
    .sqrt()
        < 2.0 * end.magnitude
    {
        return Err(());
    }

    // get the tangent length via some simple trigonometry
    route_csc.tangent.magnitude =
        ((route_csc.end.circle.center.x - route_csc.start.circle.center.x).powi(2)
            + (route_csc.end.circle.center.y - route_csc.start.circle.center.y).powi(2)
            - (2.0 * end.magnitude).powi(2))
        .sqrt();

    // tangent middle is the same as the middle of the straight from the center of the start
    let tangent_middle = route_csc
        .end
        .circle
        .center
        .lerp(route_csc.start.circle.center, 0.5);

    // get the tangent angle
    route_csc.tangent.angle = Angle::radians(
        ((route_csc.end.circle.center.y - tangent_middle.y)
            / (route_csc.end.circle.center.x - tangent_middle.x))
            .atan()
            - (2.0 * end.magnitude / route_csc.tangent.magnitude).atan(),
    );

    // if the end circle center x value is smaller than the
    // start circle center x value
    // the angle would be π rotated so to prevent that:
    if route_csc.end.circle.center.x < route_csc.start.circle.center.x {
        route_csc.tangent.angle += Angle::pi();
    }

    // get the angle of the start circle
    route_csc.start.angle = (Angle::frac_pi_2() - route_csc.tangent.angle).positive();

    // get the tangent origin by moving the vector from the start circle center
    // along its right angle vector
    route_csc.tangent.origin = route_csc.start.circle.center
        + Rotation::new(Angle::pi() - route_csc.start.angle)
            .transform_vector(Vector2D::new(route_csc.start.circle.radius, 0.0));

    // get the angle of the end circle
    route_csc.end.angle = ((Angle::frac_pi_2() - end.angle) - route_csc.tangent.angle).positive();

    Ok(route_csc)
}

/// left straight right route
pub fn lsr(end: Vector) -> Result<RouteCSC, ()> {
    let mut route_csc = RouteCSC {
        start: CircleRoute {
            circle: Circle {
                center: Point::new(-end.magnitude, 0.0),
                radius: end.magnitude,
            },
            angle: Angle::zero(),
        },
        tangent: Vector {
            origin: Point::zero(),
            angle: Angle::zero(),
            magnitude: 0.0,
        },
        end: CircleRoute {
            circle: Circle {
                center: Point::zero(),
                radius: end.magnitude,
            },
            angle: Angle::zero(),
        },
    };

    // get the center point by adding the end vector to the end point
    // this works because the argument is the angle in positive y direction
    // not positive x direction so we dont have to rotate it here anymore
    // the angle has to be counter clockwise though (thats why 2π - end.angle)
    route_csc.end.circle.center = end.origin
        + Rotation::new(end.angle)
            .inverse()
            .transform_vector(Vector2D::new(end.magnitude, 0.0));

    // check if inside tangent can even be constructed
    if ((route_csc.end.circle.center.x - route_csc.start.circle.center.x).powi(2)
        + (route_csc.end.circle.center.y - route_csc.start.circle.center.y).powi(2))
    .sqrt()
        < 2.0 * end.magnitude
    {
        return Err(());
    }

    // get the tangent length via some simple trigonometry
    route_csc.tangent.magnitude =
        ((route_csc.end.circle.center.x - route_csc.start.circle.center.x).powi(2)
            + (route_csc.end.circle.center.y - route_csc.start.circle.center.y).powi(2)
            - (2.0 * end.magnitude).powi(2))
        .sqrt();

    // tangent middle is the same as the middle of the straight from the center of the start
    let tangent_middle = route_csc
        .end
        .circle
        .center
        .lerp(route_csc.start.circle.center, 0.5);

    // get the tangent angle
    route_csc.tangent.angle = Angle::radians(
        ((route_csc.end.circle.center.y - tangent_middle.y)
            / (route_csc.end.circle.center.x - tangent_middle.x))
            .atan()
            + (2.0 * end.magnitude / route_csc.tangent.magnitude).atan(),
    );

    // if the end circle center x value is smaller than the
    // start circle center x value
    // the angle would be 180° rotated so to prevent that:
    if route_csc.end.circle.center.x < route_csc.start.circle.center.x {
        route_csc.tangent.angle += Angle::pi();
    }

    // get the angle of the start circle
    route_csc.start.angle = (route_csc.tangent.angle - Angle::frac_pi_2()).positive();

    // get the tangent origin by moving the vector from the start circle center
    // 90° to it's own direction and the magnitude of the circle radius
    route_csc.tangent.origin = route_csc.start.circle.center
        + Rotation::new(route_csc.start.angle)
            .transform_vector(Vector2D::new(route_csc.start.circle.radius, 0.0));

    // get the angle of the end circle
    route_csc.end.angle = ((Angle::frac_pi_2() - end.angle) - route_csc.tangent.angle).positive();

    Ok(route_csc)
}
