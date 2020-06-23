//! This Crate calculates Dubins Paths
//!
//!
//! eg rsr path:
//!
//!                                              -----
//!                                            /-     \
//!                                          /-/       \
//!                                       /-- /         \
//!                                     /-    |         |
//!                                  /--      |         |
//!                                /-         |         |
//!                             /--           \         /
//!                           /-               \       X goal
//!                        /--                  \     /
//!                      /-                      -----
//!                   /--
//!                 /-
//!               -----
//!              /     \
//!             /       \
//!            /         \
//!            |         |
//!      start X         |
//!            |         |
//!            \         /
//!             \       /
//!              \     /
//!               -----
//!
//!
//! The start point is (0,0) facing in positive y-direction
//!
//! The arguments to get a path are
//!
//!   - radius: the minimum radius you can drive or respectively the radius you want to drive (f64)
//!   - end_point: the point you want to end up (Point)
//!   - end_angle: the angle you want to have in the end (Angle)
//!
//! The paths can be categorized into two subsections:
//!
//!   - The circle straight circle (CSC) paths:
//!
//!     - right straight right (rsr) paths
//!     - right straight left  (rsl) paths
//!     - left straight right  (lsr) paths
//!     - left straight left   (lsl) paths
//!
//!   note: rsr and lsl paths can be constructed on every point on the plane,
//!         rsl and lsr might return an error if the circles overlap each other,
//!         because then the path cannot be constructed
//!
//!   - The circle circle circle (CCC) paths:
//!
//!     - right left right (rlr) paths
//!     - left right left  (lrl) paths
//!
//!   note: both paths can return an error if the points are too far apart
//!
//!

use euclid::{approxeq::ApproxEq, Point2D, Rotation2D, UnknownUnit};
use thiserror::Error;

pub type Angle = euclid::Angle<f64>;
pub type Point = Point2D<f64, UnknownUnit>;
pub type Vector = euclid::Vector2D<f64, UnknownUnit>;
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
pub struct StraightPath {
    pub origin: Point,
    pub vector: Vector,
}

impl StraightPath {
    /// approximate equality to other Vector
    pub fn approx_eq(&self, other: Self) -> bool {
        ApproxEq::approx_eq(&self.vector, &other.vector)
            && ApproxEq::approx_eq(&self.origin, &other.origin)
    }
}

/// Circle vector (Circle + Angle)
#[derive(Debug, Copy, Clone)]
pub struct CirclePath {
    pub center: Point,
    pub radius: f64,
    pub angle: Angle,
}

impl CirclePath {
    ///get the length of the circle vector
    pub fn get_length(&self) -> f64 {
        self.angle.radians * self.radius
    }
    /// approximate equality to other CirclePath
    pub fn approx_eq(&self, other: Self) -> bool {
        if !ApproxEq::approx_eq(&self.center, &other.center) {
            false
        } else if !ApproxEq::approx_eq(&self.radius, &other.radius) {
            false
        } else if !(ApproxEq::approx_eq(&self.angle, &other.angle)
            || ApproxEq::approx_eq(&self.angle.signed(), &other.angle.signed()))
        {
            false
        } else {
            true
        }
    }
}

/// Route with a start Circle, a tangent straight and a end Circle
#[derive(Debug, Copy, Clone)]
pub struct RouteCSC {
    pub start: CirclePath,
    pub tangent: StraightPath,
    pub end: CirclePath,
}

/// Route with 3 Circles
#[derive(Debug, Copy, Clone)]
pub struct RouteCCC {
    pub start: CirclePath,
    pub middle: CirclePath,
    pub end: CirclePath,
}

#[derive(Debug, Copy, Clone)]
pub enum Path {
    CSC(RouteCSC),
    CCC(RouteCCC),
}

/// Route with a start Circle, a tangent straight and a end Circle
impl RouteCSC {
    /// right straight right route
    pub fn rsr(radius: f64, end_point: Point, end_angle: Angle) -> Result<Self, Error> {
        let start_center = Point::new(radius, 0.0);

        // get the center point by adding the end vector to the end point
        // this works because the argument is the angle in positive y direction
        // not positive x direction so we dont have to rotate it here anymore
        // the angle has to be counter clockwise though (thats why we use the inverse end.angle)
        let end_center = end_point
            + Rotation::new(end_angle)
                .inverse()
                .transform_vector(Vector::new(radius, 0.0));

        // get the tangent pitch which is the same as the pitch between the two
        // circle centers since our circles have the same radius
        let mut tangent_angle = Angle::radians(
            ((end_center.y - start_center.y) / (end_center.x - start_center.x)).atan(),
        );

        // if the end circle center x value is smaller than the
        // start circle center x value
        // the angle would be rotated by π so to prevent that:
        if end_center.x < start_center.x {
            tangent_angle += Angle::pi();
        }

        // get the tangent magnitude this, again, is the same as the distance
        // between the two circle centers since our circles have the same radius
        let tangent_magnitude = ((end_center.x - start_center.x).powi(2)
            + (end_center.y - start_center.y).powi(2))
        .sqrt();

        // get the angle of the start circle
        let start_angle = (Angle::frac_pi_2() - tangent_angle).positive();

        // get the tangent origin by moving the vector from the start circle center
        // π/2 to it's own direction and the magnitude of the circle radius
        let tangent_origin = start_center
            + Rotation::new(Angle::pi() - end_angle).transform_vector(Vector::new(radius, 0.0));

        // get the angle of the start circle
        // the angle where we start from the tangent equals the one we finish
        // so we can use that in here
        let end_angle = (end_angle - start_angle).positive();

        Ok(Self {
            start: CirclePath {
                center: start_center,
                radius: radius,
                angle: start_angle,
            },
            tangent: StraightPath {
                origin: tangent_origin,
                vector: Vector::from_angle_and_length(tangent_angle, tangent_magnitude),
            },
            end: CirclePath {
                center: end_center,
                radius: radius,
                angle: end_angle,
            },
        })
    }

    /// left straight left route
    pub fn lsl(radius: f64, end_point: Point, end_angle: Angle) -> Result<Self, Error> {
        let start_center = Point::new(-radius, 0.0);

        // get the center point by adding the end vector to the end point
        // we have to rotate the vector π (π/2 because the given angle is from the y axis
        // and π/2 more to not get the tangent but the vector to the center point)
        // and again we have to use the counter clockwise direction
        let end_center = end_point
            + Rotation::new(Angle::pi() - end_angle).transform_vector(Vector::new(radius, 0.0));

        // get the tangent pitch which is the same as the pitch between the two
        // circle centers since our circles have the same radius
        let mut tangent_angle = Angle::radians(
            ((end_center.y - start_center.y) / (end_center.x - start_center.x)).atan(),
        )
        .positive();

        // if the end circle center x value is smaller than the
        // start circle center x value
        // the angle would be π rotated so to prevent that:
        if end_center.x < start_center.x {
            tangent_angle = (tangent_angle + Angle::pi()).positive();
        }

        // get the tangent magnitude this, again, is the same as the distance
        // between the two circle centers since our circles have the same radius
        let tangent_magnitude = ((end_center.x - start_center.x).abs().powi(2)
            + (end_center.y - start_center.y).abs().powi(2))
        .sqrt();

        // get the angle of the start circle
        let start_angle = (tangent_angle - Angle::frac_pi_2()).positive();

        // get the tangent origin by moving the vector from the start circle center
        // π/2 to it's own direction and the magnitude of the circle radius
        let tangent_origin =
            start_center + Rotation::new(start_angle).transform_vector(Vector::new(radius, 0.0));

        // get the angle of the start circle
        // the angle where we start from the tangent equals the one we finish
        // so we can use that in here
        let end_angle = (end_angle - start_angle).positive();

        Ok(Self {
            start: CirclePath {
                center: start_center,
                radius: radius,
                angle: start_angle,
            },
            tangent: StraightPath {
                origin: tangent_origin,
                vector: Vector::from_angle_and_length(tangent_angle, tangent_magnitude),
            },
            end: CirclePath {
                center: end_center,
                radius: radius,
                angle: end_angle,
            },
        })
    }

    /// right straight left route
    pub fn rsl(radius: f64, end_point: Point, end_angle: Angle) -> Result<Self, Error> {
        let start_center = Point::new(radius, 0.0);

        // get the center point by adding the end vector to the end point
        // we have to rotate the vector π (π/2 because the given angle is from the y axis
        // and π/2 more to not get the tangent but the vector to the center point)
        // and again we have to use the counter clockwise direction
        let end_center = end_point
            + Rotation::new(Angle::pi() - end_angle).transform_vector(Vector::new(radius, 0.0));

        // check if inside tangent can even be constructed
        if ((end_center.x - start_center.x).powi(2) + (end_center.y - start_center.y).powi(2))
            .sqrt()
            < 2.0 * radius
        {
            return Err(Error::CirclesTooClose);
        }

        // get the tangent length via some simple trigonometry
        let tangent_magnitude = ((end_center.x - start_center.x).powi(2)
            + (end_center.y - start_center.y).powi(2)
            - (2.0 * radius).powi(2))
        .sqrt();

        // tangent middle is the same as the middle of the straight from the center of the start
        let tangent_middle = end_center.lerp(start_center, 0.5);

        // get the tangent angle
        let mut tangent_angle = Angle::radians(
            ((end_center.y - tangent_middle.y) / (end_center.x - tangent_middle.x)).atan()
                - (2.0 * radius / tangent_magnitude).atan(),
        );

        // if the end circle center x value is smaller than the
        // start circle center x value
        // the angle would be π rotated so to prevent that:
        if end_center.x < start_center.x {
            tangent_angle += Angle::pi();
        }

        // get the angle of the start circle
        let start_angle = (Angle::frac_pi_2() - tangent_angle).positive();

        // get the tangent origin by moving the vector from the start circle center
        // along its right angle vector
        let tangent_origin = start_center
            + Rotation::new(Angle::pi() - start_angle).transform_vector(Vector::new(radius, 0.0));

        // get the angle of the end circle
        let end_angle = ((Angle::frac_pi_2() - end_angle) - tangent_angle).positive();

        Ok(Self {
            start: CirclePath {
                center: start_center,
                radius: radius,
                angle: start_angle,
            },
            tangent: StraightPath {
                origin: tangent_origin,
                vector: Vector::from_angle_and_length(tangent_angle, tangent_magnitude),
            },
            end: CirclePath {
                center: end_center,
                radius: radius,
                angle: end_angle,
            },
        })
    }

    /// left straight right route
    pub fn lsr(radius: f64, end_point: Point, end_angle: Angle) -> Result<Self, Error> {
        let start_center = Point::new(-radius, 0.0);

        // get the center point by adding the end vector to the end point
        // this works because the argument is the angle in positive y direction
        // not positive x direction so we dont have to rotate it here anymore
        // the angle has to be counter clockwise though (thats why 2π - end.angle)
        let end_center = end_point
            + Rotation::new(end_angle)
                .inverse()
                .transform_vector(Vector::new(radius, 0.0));

        // check if inside tangent can even be constructed
        if ((end_center.x - start_center.x).powi(2) + (end_center.y - start_center.y).powi(2))
            .sqrt()
            < 2.0 * radius
        {
            return Err(Error::CirclesTooClose);
        }

        // get the tangent length via some simple trigonometry
        let tangent_magnitude = ((end_center.x - start_center.x).powi(2)
            + (end_center.y - start_center.y).powi(2)
            - (2.0 * radius).powi(2))
        .sqrt();

        // tangent middle is the same as the middle of the straight from the center of the start
        let tangent_middle = end_center.lerp(start_center, 0.5);

        // get the tangent angle
        let mut tangent_angle = Angle::radians(
            ((end_center.y - tangent_middle.y) / (end_center.x - tangent_middle.x)).atan()
                + (2.0 * radius / tangent_magnitude).atan(),
        );

        // if the end circle center x value is smaller than the
        // start circle center x value
        // the angle would rotated by π so to prevent that:
        if end_center.x < start_center.x {
            tangent_angle += Angle::pi();
        }

        // get the angle of the start circle
        let start_angle = (tangent_angle - Angle::frac_pi_2()).positive();

        // get the tangent origin by moving the vector from the start circle center
        // π/2 to it's own direction and the magnitude of the circle radius
        let tangent_origin =
            start_center + Rotation::new(start_angle).transform_vector(Vector::new(radius, 0.0));

        // get the angle of the end circle
        let end_angle = ((Angle::frac_pi_2() - end_angle) - tangent_angle).positive();

        Ok(Self {
            start: CirclePath {
                center: start_center,
                radius: radius,
                angle: start_angle,
            },
            tangent: StraightPath {
                origin: tangent_origin,
                vector: Vector::from_angle_and_length(tangent_angle, tangent_magnitude),
            },
            end: CirclePath {
                center: end_center,
                radius: radius,
                angle: end_angle,
            },
        })
    }

    /// get the length of the path
    pub fn get_length(&self) -> f64 {
        self.start.get_length() + self.tangent.vector.length() + self.end.get_length()
    }

    /// get the shortest circle straight circle route
    pub fn get_shortest(radius: f64, end_point: Point, end_angle: Angle) -> Result<Self, Error> {
        let mut route_csc;

        let route_rsr = Self::rsr(radius, end_point, end_angle).unwrap();
        let route_lsl = Self::rsr(radius, end_point, end_angle).unwrap();
        let route_lsr = Self::rsr(radius, end_point, end_angle);
        let route_rsl = Self::rsr(radius, end_point, end_angle);

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
    pub fn rlr(radius: f64, end_point: Point, end_angle: Angle) -> Result<Self, Error> {
        let start_center = Point::new(radius, 0.0);

        // get the center point by adding the end vector to the end point
        // this works because the argument is the angle in positive y direction
        // not positive x direction so we dont have to rotate it here anymore
        // the angle has to be counter clockwise though (thats why we use the inverse end.angle)
        let end_center = end_point
            + Rotation::new(end_angle)
                .inverse()
                .transform_vector(Vector::new(radius, 0.0));

        // check if path can be constructed or if the circles are too far apart
        if ((end_center.x - start_center.x).powi(2) + (end_center.y - start_center.y).powi(2))
            .sqrt()
            > (4.0 * radius)
        {
            return Err(Error::CirclesTooFarApart);
        }

        let vector_start_center_middle_center: Vector;

        let middle_center = {
            let vector_start_center_end_center =
                Vector::new(end_center.x - start_center.x, end_center.y - start_center.y);

            let vector_start_center_middle_center_angle =
                vector_start_center_end_center.angle_from_x_axis().radians
                    + (vector_start_center_end_center.length() / (4.0 * radius)).acos();

            vector_start_center_middle_center = Vector::new(
                (2.0 * radius) * vector_start_center_middle_center_angle.cos(),
                (2.0 * radius) * vector_start_center_middle_center_angle.sin(),
            );

            Point::new(
                start_center.x + vector_start_center_middle_center.x,
                start_center.y + vector_start_center_middle_center.y,
            )
        };

        let vector_middle_center_end_center = Vector::new(
            end_center.x - middle_center.x,
            end_center.y - middle_center.y,
        );

        let start_angle =
            (Angle::pi() - vector_start_center_middle_center.angle_from_x_axis()).positive();

        let middle_angle = Rotation::new(Angle::pi())
            .transform_vector(vector_start_center_middle_center)
            .angle_to(vector_middle_center_end_center)
            .positive();

        let end_angle = Rotation::new(Angle::pi())
            .transform_vector(vector_middle_center_end_center)
            .angle_to(Vector::new(
                end_point.x - end_center.x,
                end_point.y - end_center.y,
            ))
            .positive();

        Ok(Self {
            start: CirclePath {
                center: start_center,
                radius: radius,
                angle: start_angle,
            },
            middle: CirclePath {
                center: middle_center,
                radius: radius,
                angle: middle_angle,
            },
            end: CirclePath {
                center: end_center,
                radius: radius,
                angle: end_angle,
            },
        })
    }

    /// left right left route (not working yet)
    pub fn lrl(radius: f64, end_point: Point, end_angle: Angle) -> Result<Self, Error> {
        let start_center = Point::new(-radius, 0.0);

        // get the center point by adding the end vector to the end point
        // we have to rotate the vector π (π/2 because the given angle is from the y axis
        // and π/2 more to not get the tangent but the vector to the center point)
        // and again we have to use the counter clockwise direction
        let end_center = end_point
            + Rotation::new(Angle::pi() - end_angle).transform_vector(Vector::new(radius, 0.0));

        // check if path can be constructed or if the circles are too far apart
        if ((end_center.x - start_center.x).powi(2) + (end_center.y - start_center.y).powi(2))
            .sqrt()
            > (4.0 * radius)
        {
            return Err(Error::CirclesTooFarApart);
        }

        let vector_start_center_middle_center: Vector;

        let middle_center = {
            let vector_start_center_end_center =
                Vector::new(end_center.x - start_center.x, end_center.y - start_center.y);

            let vector_start_center_middle_center_angle =
                vector_start_center_end_center.angle_from_x_axis().radians
                    - (vector_start_center_end_center.length() / (4.0 * radius)).acos();

            vector_start_center_middle_center = Vector::new(
                (2.0 * radius) * vector_start_center_middle_center_angle.cos(),
                (2.0 * radius) * vector_start_center_middle_center_angle.sin(),
            );

            Point::new(
                start_center.x + vector_start_center_middle_center.x,
                start_center.y + vector_start_center_middle_center.y,
            )
        };

        let vector_middle_center_end_center = Vector::new(
            end_center.x - middle_center.x,
            end_center.y - middle_center.y,
        );

        let start_angle = (vector_start_center_middle_center.angle_from_x_axis()).positive();

        let middle_angle = vector_middle_center_end_center
            .angle_to(
                Rotation::new(Angle::pi()).transform_vector(vector_start_center_middle_center),
            )
            .positive();

        let end_angle = Vector::new(end_point.x - end_center.x, end_point.y - end_center.y)
            .angle_to(Rotation::new(Angle::pi()).transform_vector(vector_middle_center_end_center))
            .positive();

        Ok(Self {
            start: CirclePath {
                center: start_center,
                radius: radius,
                angle: start_angle,
            },
            middle: CirclePath {
                center: middle_center,
                radius: radius,
                angle: middle_angle,
            },
            end: CirclePath {
                center: end_center,
                radius: radius,
                angle: end_angle,
            },
        })
    }

    /// get the length of the path
    pub fn get_length(&self) -> f64 {
        self.start.get_length() + self.middle.get_length() + self.end.get_length()
    }

    /// get the shortest circle circle circle route
    pub fn get_shortest(radius: f64, end_point: Point, end_angle: Angle) -> Result<Self, Error> {
        let route_rlr = Self::rlr(radius, end_point, end_angle);
        let route_lrl = Self::lrl(radius, end_point, end_angle);

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
pub fn get_shortest(radius: f64, end_point: Point, end_angle: Angle) -> Path {
    let route_csc = RouteCSC::get_shortest(radius, end_point, end_angle).unwrap();
    let route_ccc = RouteCCC::get_shortest(radius, end_point, end_angle);
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
