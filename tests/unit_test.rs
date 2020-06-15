#[cfg(test)]
mod tests {
    use dubins_path::*;

    #[test]
    #[should_panic]
    fn test_overlapping_circles_rsl() {
        let radius = 0.5;
        let test_point = Vector {
            origin: Point::new(radius, 0.0),
            angle: Angle::zero(),
            magnitude: radius,
        };
        RouteCSC::rsl(test_point).unwrap();
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
        RouteCSC::lsr(test_point).unwrap();
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

            let result_rsr = RouteCSC::rsr(test_point).unwrap();

            assert!(result_rsr.start.approx_eq(expected_result_rsr.start));
            assert!(result_rsr.tangent.approx_eq(expected_result_rsr.tangent));
            assert!(result_rsr.end.approx_eq(expected_result_rsr.end));
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

            let result_rsl = RouteCSC::rsl(test_point).unwrap();

            assert!(result_rsl.start.approx_eq(expected_result_rsl.start));
            assert!(result_rsl.tangent.approx_eq(expected_result_rsl.tangent));
            assert!(result_rsl.end.approx_eq(expected_result_rsl.end));
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

            let result_lsl = RouteCSC::lsl(test_point).unwrap();

            assert!(result_lsl.start.approx_eq(expected_result_lsl.start));
            assert!(result_lsl.tangent.approx_eq(expected_result_lsl.tangent));
            assert!(result_lsl.end.approx_eq(expected_result_lsl.end));
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

            let result_lsr = RouteCSC::lsr(test_point).unwrap();

            assert!(result_lsr.start.approx_eq(expected_result_lsr.start));
            assert!(result_lsr.tangent.approx_eq(expected_result_lsr.tangent));
            assert!(result_lsr.end.approx_eq(expected_result_lsr.end));
        }
    }
}
