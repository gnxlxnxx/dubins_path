//! Call all functions with a Vector as argument the vector should contain:
//!  - the end point as origin
//!  - the end angle as angle in degrees in clockwise direction (eg. 0° facing north, 90° (π/2) facing east, ...)
//!  - the circle radius as magnitude
//!
//! Start Vector is in the origin facing in positive y-direction
//!
//! Every struct defined here is 2 dimensional and uses f64

use euclid::{Point2D, Rotation2D, UnknownUnit};
use thiserror::Error;

pub type Angle = euclid::Angle<f64>;
pub type Point = Point2D<f64, UnknownUnit>;
type Vector2D = euclid::Vector2D<f64, UnknownUnit>;
type Rotation = Rotation2D<f64, UnknownUnit, UnknownUnit>;

#[derive(Debug, Error)]
pub enum Error {
    #[error("inside tangent cannot be constructed (circles too close together)")]
    CirclesTooClose,
    #[error("ccc path cannot be constructed (circles too far apart)")]
    CirclesTooFarApart,
}

/// Vector with origin, angle and magnitude
#[derive(Debug, Copy, Clone)]
pub struct Vector {
    pub origin: Point,
    pub angle: Angle,
    pub magnitude: f64,
}

/// Circle vector (Circle + Angle)
#[derive(Debug, Copy, Clone)]
pub struct CircleVector {
    pub center: Point,
    pub radius: f64,
    pub angle: Angle,
}

impl CircleVector {
    ///get the length of the circle vector
    pub fn get_length(&self) -> f64 {
        self.angle.radians * self.radius
    }
}

/// Route with a start Circle, a tangent straight and a end Circle
#[derive(Debug, Copy, Clone)]
pub struct RouteCSC {
    pub start: CircleVector,
    pub tangent: Vector,
    pub end: CircleVector,
}

/// Route with 3 Circles
#[derive(Debug, Copy, Clone)]
pub struct RouteCCC {
    pub start: CircleVector,
    pub middle: CircleVector,
    pub end: CircleVector,
}

#[derive(Debug, Copy, Clone)]
pub enum Path {
    CSC(RouteCSC),
    CCC(RouteCCC),
}

/// Route with a start Circle, a tangent straight and a end Circle
impl RouteCSC {
    /// right straight right route
    pub fn rsr(end: Vector) -> Result<Self, Error> {
        let mut route_csc = RouteCSC {
            start: CircleVector {
                center: Point::new(end.magnitude, 0.0),
                radius: end.magnitude,
                angle: Angle::zero(),
            },
            tangent: Vector {
                origin: Point::zero(),
                angle: Angle::zero(),
                magnitude: 0.0,
            },
            end: CircleVector {
                center: Point::zero(),
                radius: end.magnitude,
                angle: Angle::zero(),
            },
        };

        // get the center point by adding the end vector to the end point
        // this works because the argument is the angle in positive y direction
        // not positive x direction so we dont have to rotate it here anymore
        // the angle has to be counter clockwise though (thats why we use the inverse end.angle)
        route_csc.end.center = end.origin
            + Rotation::new(end.angle)
                .inverse()
                .transform_vector(Vector2D::new(end.magnitude, 0.0));

        // get the tangent pitch which is the same as the pitch between the two
        // circle centers since our circles have the same radius
        route_csc.tangent.angle = Angle::radians(
            ((route_csc.end.center.y - route_csc.start.center.y)
                / (route_csc.end.center.x - route_csc.start.center.x))
                .atan(),
        );

        // if the end circle center x value is smaller than the
        // start circle center x value
        // the angle would be rotated by π so to prevent that:
        if route_csc.end.center.x < route_csc.start.center.x {
            route_csc.tangent.angle += Angle::pi();
        }

        // get the tangent magnitude this, again, is the same as the distance
        // between the two circle centers since our circles have the same radius
        route_csc.tangent.magnitude = ((route_csc.end.center.x - route_csc.start.center.x).powi(2)
            + (route_csc.end.center.y - route_csc.start.center.y).powi(2))
        .sqrt();

        // get the angle of the start circle
        route_csc.start.angle = (Angle::frac_pi_2() - route_csc.tangent.angle).positive();

        // get the tangent origin by moving the vector from the start circle center
        // π/2 to it's own direction and the magnitude of the circle radius
        route_csc.tangent.origin = route_csc.start.center
            + Rotation::new(Angle::pi() - end.angle)
                .transform_vector(Vector2D::new(route_csc.start.radius, 0.0));

        // get the angle of the start circle
        // the angle where we start from the tangent equals the one we finish
        // so we can use that in here
        route_csc.end.angle = (end.angle - route_csc.start.angle).positive();

        Ok(route_csc)
    }

    /// left straight left route
    pub fn lsl(end: Vector) -> Result<Self, Error> {
        let mut route_csc = RouteCSC {
            start: CircleVector {
                center: Point::new(-end.magnitude, 0.0),
                radius: end.magnitude,
                angle: Angle::zero(),
            },
            tangent: Vector {
                origin: Point::zero(),
                angle: Angle::zero(),
                magnitude: 0.0,
            },
            end: CircleVector {
                center: Point::zero(),
                radius: end.magnitude,
                angle: Angle::zero(),
            },
        };

        // get the center point by adding the end vector to the end point
        // we have to rotate the vector π (π/2 because the given angle is from the y axis
        // and π/2 more to not get the tangent but the vector to the center point)
        // and again we have to use the counter clockwise direction
        route_csc.end.center = end.origin
            + Rotation::new(Angle::pi() - end.angle)
                .transform_vector(Vector2D::new(end.magnitude, 0.0));

        // get the tangent pitch which is the same as the pitch between the two
        // circle centers since our circles have the same radius
        route_csc.tangent.angle = Angle::radians(
            ((route_csc.end.center.y - route_csc.start.center.y)
                / (route_csc.end.center.x - route_csc.start.center.x))
                .atan(),
        )
        .positive();

        // if the end circle center x value is smaller than the
        // start circle center x value
        // the angle would be π rotated so to prevent that:
        if route_csc.end.center.x < route_csc.start.center.x {
            route_csc.tangent.angle = (route_csc.tangent.angle + Angle::pi()).positive();
        }

        // get the tangent magnitude this, again, is the same as the distance
        // between the two circle centers since our circles have the same radius
        route_csc.tangent.magnitude = ((route_csc.end.center.x - route_csc.start.center.x)
            .abs()
            .powi(2)
            + (route_csc.end.center.y - route_csc.start.center.y)
                .abs()
                .powi(2))
        .sqrt();

        // get the angle of the start circle
        route_csc.start.angle = (route_csc.tangent.angle - Angle::frac_pi_2()).positive();

        // get the tangent origin by moving the vector from the start circle center
        // π/2 to it's own direction and the magnitude of the circle radius
        route_csc.tangent.origin = route_csc.start.center
            + Rotation::new(route_csc.start.angle)
                .transform_vector(Vector2D::new(route_csc.start.radius, 0.0));

        // get the angle of the start circle
        // the angle where we start from the tangent equals the one we finish
        // so we can use that in here
        route_csc.end.angle = (end.angle - route_csc.start.angle).positive();

        Ok(route_csc)
    }

    /// right straight left route
    pub fn rsl(end: Vector) -> Result<Self, Error> {
        let mut route_csc = RouteCSC {
            start: CircleVector {
                center: Point::new(end.magnitude, 0.0),
                radius: end.magnitude,
                angle: Angle::zero(),
            },
            tangent: Vector {
                origin: Point::zero(),
                angle: Angle::zero(),
                magnitude: 0.0,
            },
            end: CircleVector {
                center: Point::zero(),
                radius: end.magnitude,
                angle: Angle::zero(),
            },
        };

        // get the center point by adding the end vector to the end point
        // we have to rotate the vector π (π/2 because the given angle is from the y axis
        // and π/2 more to not get the tangent but the vector to the center point)
        // and again we have to use the counter clockwise direction
        route_csc.end.center = end.origin
            + Rotation::new(Angle::pi() - end.angle)
                .transform_vector(Vector2D::new(end.magnitude, 0.0));

        // check if inside tangent can even be constructed
        if ((route_csc.end.center.x - route_csc.start.center.x).powi(2)
            + (route_csc.end.center.y - route_csc.start.center.y).powi(2))
        .sqrt()
            < 2.0 * end.magnitude
        {
            return Err(Error::CirclesTooClose);
        }

        // get the tangent length via some simple trigonometry
        route_csc.tangent.magnitude = ((route_csc.end.center.x - route_csc.start.center.x).powi(2)
            + (route_csc.end.center.y - route_csc.start.center.y).powi(2)
            - (2.0 * end.magnitude).powi(2))
        .sqrt();

        // tangent middle is the same as the middle of the straight from the center of the start
        let tangent_middle = route_csc.end.center.lerp(route_csc.start.center, 0.5);

        // get the tangent angle
        route_csc.tangent.angle = Angle::radians(
            ((route_csc.end.center.y - tangent_middle.y)
                / (route_csc.end.center.x - tangent_middle.x))
                .atan()
                - (2.0 * end.magnitude / route_csc.tangent.magnitude).atan(),
        );

        // if the end circle center x value is smaller than the
        // start circle center x value
        // the angle would be π rotated so to prevent that:
        if route_csc.end.center.x < route_csc.start.center.x {
            route_csc.tangent.angle += Angle::pi();
        }

        // get the angle of the start circle
        route_csc.start.angle = (Angle::frac_pi_2() - route_csc.tangent.angle).positive();

        // get the tangent origin by moving the vector from the start circle center
        // along its right angle vector
        route_csc.tangent.origin = route_csc.start.center
            + Rotation::new(Angle::pi() - route_csc.start.angle)
                .transform_vector(Vector2D::new(route_csc.start.radius, 0.0));

        // get the angle of the end circle
        route_csc.end.angle =
            ((Angle::frac_pi_2() - end.angle) - route_csc.tangent.angle).positive();

        Ok(route_csc)
    }

    /// left straight right route
    pub fn lsr(end: Vector) -> Result<Self, Error> {
        let mut route_csc = RouteCSC {
            start: CircleVector {
                center: Point::new(-end.magnitude, 0.0),
                radius: end.magnitude,
                angle: Angle::zero(),
            },
            tangent: Vector {
                origin: Point::zero(),
                angle: Angle::zero(),
                magnitude: 0.0,
            },
            end: CircleVector {
                center: Point::zero(),
                radius: end.magnitude,
                angle: Angle::zero(),
            },
        };

        // get the center point by adding the end vector to the end point
        // this works because the argument is the angle in positive y direction
        // not positive x direction so we dont have to rotate it here anymore
        // the angle has to be counter clockwise though (thats why 2π - end.angle)
        route_csc.end.center = end.origin
            + Rotation::new(end.angle)
                .inverse()
                .transform_vector(Vector2D::new(end.magnitude, 0.0));

        // check if inside tangent can even be constructed
        if ((route_csc.end.center.x - route_csc.start.center.x).powi(2)
            + (route_csc.end.center.y - route_csc.start.center.y).powi(2))
        .sqrt()
            < 2.0 * end.magnitude
        {
            return Err(Error::CirclesTooClose);
        }

        // get the tangent length via some simple trigonometry
        route_csc.tangent.magnitude = ((route_csc.end.center.x - route_csc.start.center.x).powi(2)
            + (route_csc.end.center.y - route_csc.start.center.y).powi(2)
            - (2.0 * end.magnitude).powi(2))
        .sqrt();

        // tangent middle is the same as the middle of the straight from the center of the start
        let tangent_middle = route_csc.end.center.lerp(route_csc.start.center, 0.5);

        // get the tangent angle
        route_csc.tangent.angle = Angle::radians(
            ((route_csc.end.center.y - tangent_middle.y)
                / (route_csc.end.center.x - tangent_middle.x))
                .atan()
                + (2.0 * end.magnitude / route_csc.tangent.magnitude).atan(),
        );

        // if the end circle center x value is smaller than the
        // start circle center x value
        // the angle would rotated by π so to prevent that:
        if route_csc.end.center.x < route_csc.start.center.x {
            route_csc.tangent.angle += Angle::pi();
        }

        // get the angle of the start circle
        route_csc.start.angle = (route_csc.tangent.angle - Angle::frac_pi_2()).positive();

        // get the tangent origin by moving the vector from the start circle center
        // π/2 to it's own direction and the magnitude of the circle radius
        route_csc.tangent.origin = route_csc.start.center
            + Rotation::new(route_csc.start.angle)
                .transform_vector(Vector2D::new(route_csc.start.radius, 0.0));

        // get the angle of the end circle
        route_csc.end.angle =
            ((Angle::frac_pi_2() - end.angle) - route_csc.tangent.angle).positive();

        Ok(route_csc)
    }

    /// get the length of the path
    pub fn get_length(&self) -> f64 {
        self.start.get_length() + self.tangent.magnitude + self.end.get_length()
    }

    /// get the shortest circle straight circle route
    pub fn get_shortest(end: Vector) -> Result<Self, Error> {
        let mut route_csc;

        let route_rsr = Self::rsr(end).unwrap();
        let route_lsl = Self::rsr(end).unwrap();
        let route_lsr = Self::rsr(end);
        let route_rsl = Self::rsr(end);

        route_csc = route_rsr;
        if route_lsl.get_length() < route_csc.get_length() {
            route_csc = route_lsl;
        }
        if let Ok(route_lsr) = route_lsr {
            if route_lsr.get_length() < route_csc.get_length() {
                route_csc = route_lsr;
            }
        }
        if let Ok(route_rsl) = route_rsl {
            if route_rsl.get_length() < route_csc.get_length() {
                route_csc = route_rsl;
            }
        }

        Ok(route_csc)
    }
}

/// Route with 3 Circles
impl RouteCCC {
    /// right left right route (not working yet)
    pub fn rlr(end: Vector) -> Result<Self, Error> {
        let mut route_ccc = RouteCCC {
            start: CircleVector {
                center: Point::new(end.magnitude, 0.0),
                radius: end.magnitude,
                angle: Angle::zero(),
            },
            middle: CircleVector {
                center: Point::zero(),
                radius: 0.0,
                angle: Angle::zero(),
            },
            end: CircleVector {
                center: Point::zero(),
                radius: end.magnitude,
                angle: Angle::zero(),
            },
        };

        // get the center point by adding the end vector to the end point
        // this works because the argument is the angle in positive y direction
        // not positive x direction so we dont have to rotate it here anymore
        // the angle has to be counter clockwise though (thats why we use the inverse end.angle)
        route_ccc.end.center = end.origin
            + Rotation::new(end.angle)
                .inverse()
                .transform_vector(Vector2D::new(end.magnitude, 0.0));

        // check if path can be constructed or if the circles are too far apart
        if ((route_ccc.end.center.x - route_ccc.start.center.x).powi(2)
            + (route_ccc.end.center.y - route_ccc.start.center.y).powi(2))
        .sqrt()
            > (4.0 * end.magnitude)
        {
            return Err(Error::CirclesTooFarApart);
        }

        let vector_start_center_middle_center: Vector2D;

        route_ccc.middle.center = {
            let vector_start_center_end_center = Vector2D::new(
                route_ccc.end.center.x - route_ccc.start.center.x,
                route_ccc.end.center.y - route_ccc.start.center.y,
            );

            let vector_start_center_end_center_magnitude =
                (vector_start_center_end_center.x.powi(2)
                    + vector_start_center_end_center.y.powi(2))
                .sqrt();

            let vector_start_center_middle_center_angle =
                vector_start_center_end_center.angle_from_x_axis().radians
                    + (vector_start_center_end_center_magnitude / (4.0 * end.magnitude)).acos();

            vector_start_center_middle_center = Vector2D::new(
                (2.0 * end.magnitude) * vector_start_center_middle_center_angle.cos(),
                (2.0 * end.magnitude) * vector_start_center_middle_center_angle.sin(),
            );

            Point::new(
                route_ccc.start.center.x + vector_start_center_middle_center.x,
                route_ccc.start.center.y + vector_start_center_middle_center.y,
            )
        };

        let vector_middle_center_end_center = Vector2D::new(
            route_ccc.end.center.x - route_ccc.middle.center.x,
            route_ccc.end.center.y - route_ccc.middle.center.y,
        );

        route_ccc.start.angle =
            (Angle::pi() - vector_start_center_middle_center.angle_from_x_axis()).positive();

        route_ccc.middle.angle = Rotation::new(Angle::pi())
            .transform_vector(vector_start_center_middle_center)
            .angle_to(vector_middle_center_end_center)
            .positive();

        route_ccc.end.angle = Rotation::new(Angle::pi())
            .transform_vector(vector_middle_center_end_center)
            .angle_to(Vector2D::new(
                end.origin.x - route_ccc.end.center.x,
                end.origin.y - route_ccc.end.center.y,
            ))
            .positive();

        Ok(route_ccc)
    }

    /// left right left route (not working yet)
    pub fn lrl(end: Vector) -> Result<Self, Error> {
        let mut route_ccc = RouteCCC {
            start: CircleVector {
                center: Point::new(-end.magnitude, 0.0),
                radius: end.magnitude,
                angle: Angle::zero(),
            },
            middle: CircleVector {
                center: Point::zero(),
                radius: 0.0,
                angle: Angle::zero(),
            },
            end: CircleVector {
                center: Point::zero(),
                radius: end.magnitude,
                angle: Angle::zero(),
            },
        };

        // get the center point by adding the end vector to the end point
        // we have to rotate the vector π (π/2 because the given angle is from the y axis
        // and π/2 more to not get the tangent but the vector to the center point)
        // and again we have to use the counter clockwise direction
        route_ccc.end.center = end.origin
            + Rotation::new(Angle::pi() - end.angle)
                .transform_vector(Vector2D::new(end.magnitude, 0.0));

        // check if path can be constructed or if the circles are too far apart
        if ((route_ccc.end.center.x - route_ccc.start.center.x).powi(2)
            + (route_ccc.end.center.y - route_ccc.start.center.y).powi(2))
        .sqrt()
            > (4.0 * end.magnitude)
        {
            return Err(Error::CirclesTooFarApart);
        }

        let vector_start_center_middle_center: Vector2D;

        route_ccc.middle.center = {
            let vector_start_center_end_center = Vector2D::new(
                route_ccc.end.center.x - route_ccc.start.center.x,
                route_ccc.end.center.y - route_ccc.start.center.y,
            );

            let vector_start_center_end_center_magnitude =
                (vector_start_center_end_center.x.powi(2)
                    + vector_start_center_end_center.y.powi(2))
                .sqrt();

            let vector_start_center_middle_center_angle =
                vector_start_center_end_center.angle_from_x_axis().radians
                    - (vector_start_center_end_center_magnitude / (4.0 * end.magnitude)).acos();

            vector_start_center_middle_center = Vector2D::new(
                (2.0 * end.magnitude) * vector_start_center_middle_center_angle.cos(),
                (2.0 * end.magnitude) * vector_start_center_middle_center_angle.sin(),
            );

            Point::new(
                route_ccc.start.center.x + vector_start_center_middle_center.x,
                route_ccc.start.center.y + vector_start_center_middle_center.y,
            )
        };

        // TODO: Get the angles

        let vector_middle_center_end_center = Vector2D::new(
            route_ccc.end.center.x - route_ccc.middle.center.x,
            route_ccc.end.center.y - route_ccc.middle.center.y,
        );

        route_ccc.start.angle = (vector_start_center_middle_center.angle_from_x_axis()).positive();

        route_ccc.middle.angle = vector_middle_center_end_center
            .angle_to(
                Rotation::new(Angle::pi()).transform_vector(vector_start_center_middle_center),
            )
            .positive();

        route_ccc.end.angle = Vector2D::new(
            end.origin.x - route_ccc.end.center.x,
            end.origin.y - route_ccc.end.center.y,
        )
        .angle_to(Rotation::new(Angle::pi()).transform_vector(vector_middle_center_end_center))
        .positive();

        Ok(route_ccc)
    }

    /// get the length of the path
    pub fn get_length(&self) -> f64 {
        self.start.get_length() + self.middle.get_length() + self.end.get_length()
    }

    /// get the shortest circle circle circle route
    pub fn get_shortest(end: Vector) -> Result<Self, Error> {
        let route_rlr = Self::rlr(end);
        let route_lrl = Self::lrl(end);

        if let Ok(route_rlr) = route_rlr {
            if let Ok(route_lrl) = route_lrl {
                if route_rlr.get_length() < route_lrl.get_length() {
                    Ok(route_rlr)
                } else {
                    Ok(route_lrl)
                }
            } else {
                Ok(route_rlr)
            }
        } else if let Ok(route_lrl) = route_lrl {
            Ok(route_lrl)
        } else {
            Err(Error::CirclesTooFarApart)
        }
    }
}

/// get the shortest path
pub fn get_shortest(end: Vector) -> Path {
    let route_csc = RouteCSC::get_shortest(end).unwrap();
    let route_ccc = RouteCCC::get_shortest(end);
    if let Ok(route_ccc) = route_ccc {
        if route_ccc.get_length() < route_csc.get_length() {
            Path::CCC(route_ccc)
        } else {
            Path::CSC(route_csc)
        }
    } else {
        Path::CSC(route_csc)
    }
}
