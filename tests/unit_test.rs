#[cfg(test)]
mod tests {
    use dubins_path::*;

    const ERROR: f64 = 0.000000000000001;

    fn circle_in_error_margin(
        result: CircleVector,
        expected_result: CircleVector,
    ) -> Result<(), ()> {
        if result.center.x - expected_result.center.x > ERROR {
            return Err(());
        }
        if result.center.x - expected_result.center.x < -ERROR {
            return Err(());
        }
        if result.center.y - expected_result.center.y > ERROR {
            return Err(());
        }
        if result.center.y - expected_result.center.y < -ERROR {
            return Err(());
        }
        if result.radius - expected_result.radius > ERROR {
            return Err(());
        }
        if result.radius - expected_result.radius < -ERROR {
            return Err(());
        }
        if result.angle.radians - expected_result.angle.radians > ERROR
            && result.angle.signed().radians - expected_result.angle.signed().radians > ERROR
        {
            return Err(());
        }

        if result.angle.radians - expected_result.angle.radians < -ERROR
            && result.angle.signed().radians - expected_result.angle.signed().radians < -ERROR
        {
            return Err(());
        }
        Ok(())
    }

    fn vector_in_error_margin(result: Vector, expected_result: Vector) -> Result<(), ()> {
        if result.origin.x - expected_result.origin.x > ERROR {
            return Err(());
        }
        if result.origin.x - expected_result.origin.x < -ERROR {
            return Err(());
        }
        if result.origin.y - expected_result.origin.y > ERROR {
            return Err(());
        }
        if result.origin.y - expected_result.origin.y < -ERROR {
            return Err(());
        }
        if result.angle.radians - expected_result.angle.radians > ERROR
            || result.angle.signed().radians - expected_result.angle.signed().radians > ERROR
        {
            return Err(());
        }
        if result.angle.radians - expected_result.angle.radians < -ERROR
            || result.angle.signed().radians - expected_result.angle.signed().radians < -ERROR
        {
            return Err(());
        }
        if result.magnitude - expected_result.magnitude > ERROR {
            return Err(());
        }
        if result.magnitude - expected_result.magnitude < -ERROR {
            return Err(());
        }
        Ok(())
    }

    #[test]
    #[should_panic]
    fn test_overlapping_circles_rsl() {
        let radius = 0.5;
        let test_point = Vector {
            origin: Point::new(radius, 0.0),
            angle: Angle::zero(),
            magnitude: radius,
        };
        rsl(test_point).unwrap();
    }

    #[test]
    #[should_panic]
    fn test_overlapping_circles_lsr() {
        let radius = 0.5;
        let test_point = Vector {
            origin: Point::new(-radius, 0.0),
            angle: Angle::zero(),
            magnitude: radius,
        };
        lsr(test_point).unwrap();
    }

    #[test]
    fn test_points_rsr() {
        {
            let radius = 0.5;
            let test_point = Vector {
                origin: Point::new(0.0, 10.0),
                angle: Angle::zero(),
                magnitude: radius,
            };
            let expected_result_rsr = RouteCSC {
                start: CircleVector {
                    center: Point::new(0.5, 0.0),
                    radius: radius,
                    angle: Angle::zero(),
                },
                tangent: Vector {
                    origin: Point::new(0.0, 0.0),
                    angle: Angle::frac_pi_2(),
                    magnitude: 10.0,
                },
                end: CircleVector {
                    center: Point::new(0.5, 10.0),
                    radius: radius,
                    angle: Angle::zero(),
                },
            };

            let result_rsr = rsr(test_point).unwrap();

            circle_in_error_margin(result_rsr.start, expected_result_rsr.start).unwrap();
            vector_in_error_margin(result_rsr.tangent, expected_result_rsr.tangent).unwrap();
            circle_in_error_margin(result_rsr.end, expected_result_rsr.end).unwrap();
        }
    }

    #[test]
    fn test_points_rsl() {
        {
            let radius = 0.5;
            let test_point = Vector {
                origin: Point::new(0.0, 10.0),
                angle: Angle::zero(),
                magnitude: radius,
            };
            let expected_result_rsl = RouteCSC {
                start: CircleVector {
                    center: Point::new(0.5, 0.0),
                    radius: radius,
                    angle: Angle::zero(),
                },
                tangent: Vector {
                    origin: Point::new(0.0, 0.0),
                    angle: Angle::frac_pi_2(),
                    magnitude: 10.0,
                },
                end: CircleVector {
                    center: Point::new(-0.5, 10.0),
                    radius: radius,
                    angle: Angle::zero(),
                },
            };

            let result_rsl = rsl(test_point).unwrap();

            circle_in_error_margin(result_rsl.start, expected_result_rsl.start).unwrap();
            vector_in_error_margin(result_rsl.tangent, expected_result_rsl.tangent).unwrap();
            circle_in_error_margin(result_rsl.end, expected_result_rsl.end).unwrap();
        }
    }

    #[test]
    fn test_points_lsl() {
        {
            let radius = 0.5;
            let test_point = Vector {
                origin: Point::new(0.0, 10.0),
                angle: Angle::zero(),
                magnitude: radius,
            };
            let expected_result_lsl = RouteCSC {
                start: CircleVector {
                    center: Point::new(-0.5, 0.0),
                    radius: radius,
                    angle: Angle::zero(),
                },
                tangent: Vector {
                    origin: Point::new(0.0, 0.0),
                    angle: Angle::frac_pi_2(),
                    magnitude: 10.0,
                },
                end: CircleVector {
                    center: Point::new(-0.5, 10.0),
                    radius: radius,
                    angle: Angle::zero(),
                },
            };

            let result_lsl = lsl(test_point).unwrap();

            circle_in_error_margin(result_lsl.start, expected_result_lsl.start).unwrap();
            vector_in_error_margin(result_lsl.tangent, expected_result_lsl.tangent).unwrap();
            circle_in_error_margin(result_lsl.end, expected_result_lsl.end).unwrap();
        }
    }

    #[test]
    fn test_point_lsr() {
        {
            let radius = 0.5;
            let test_point = Vector {
                origin: Point::new(0.0, 10.0),
                angle: Angle::zero(),
                magnitude: radius,
            };
            let expected_result_lsr = RouteCSC {
                start: CircleVector {
                    center: Point::new(-0.5, 0.0),
                    radius: radius,
                    angle: Angle::zero(),
                },
                tangent: Vector {
                    origin: Point::new(0.0, 0.0),
                    angle: Angle::frac_pi_2(),
                    magnitude: 10.0,
                },
                end: CircleVector {
                    center: Point::new(0.5, 10.0),
                    radius: radius,
                    angle: Angle::zero(),
                },
            };

            let result_lsr = lsr(test_point).unwrap();

            circle_in_error_margin(result_lsr.start, expected_result_lsr.start).unwrap();
            vector_in_error_margin(result_lsr.tangent, expected_result_lsr.tangent).unwrap();
            circle_in_error_margin(result_lsr.end, expected_result_lsr.end).unwrap();
        }
    }
}
