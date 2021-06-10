#[cfg(test)]
mod tests {
    use dubins_path::*;

    #[test]
    #[should_panic]
    fn test_overlapping_circles_rsl() {
        let radius = 0.5;
        let end_point = Point::new(radius, 0.0);
        let end_angle = Angle::zero();
        RouteCSC::rsl(radius, end_point, end_angle).unwrap();
    }

    #[test]
    #[should_panic]
    fn test_overlapping_circles_lsr() {
        let radius = 0.5;
        let end_point = Point::new(-radius, 0.0);
        let end_angle = Angle::zero();
        RouteCSC::lsr(radius, end_point, end_angle).unwrap();
    }

    #[test]
    #[should_panic]
    fn test_far_apart_circles_rlr() {
        let radius = 0.5;
        let end_point = Point::new(7.0 * radius, 0.0);
        let end_angle = Angle::zero();
        RouteCCC::rlr(radius, end_point, end_angle).unwrap();
    }

    #[test]
    #[should_panic]
    fn test_far_apart_circles_lrl() {
        let radius = 0.5;
        let end_point = Point::new(-7.0 * radius, 0.0);
        let end_angle = Angle::zero();
        RouteCCC::lrl(radius, end_point, end_angle).unwrap();
    }

    #[test]
    fn test_points_rsr() {
        {
            let radius = 0.5;
            let end_point = Point::new(0.0, 10.0);
            let end_angle = Angle::zero();

            let expected_result_rsr = RouteCSC {
                start: CirclePath {
                    center: Point::new(0.5, 0.0),
                    radius: radius,
                    angle: Angle::zero(),
                },
                tangent: StraightPath {
                    origin: Point::new(0.0, 0.0),
                    vector: Vector::from_angle_and_length(Angle::frac_pi_2(), 10.0),
                },
                end: CirclePath {
                    center: Point::new(0.5, 10.0),
                    radius: radius,
                    angle: Angle::zero(),
                },
            };

            let result_rsr = RouteCSC::rsr(radius, end_point, end_angle).unwrap();

            assert!(result_rsr.start.approx_eq(expected_result_rsr.start));
            assert!(result_rsr.tangent.approx_eq(expected_result_rsr.tangent));
            assert!(result_rsr.end.approx_eq(expected_result_rsr.end));
        }
    }

    #[test]
    fn test_points_rsl() {
        {
            let radius = 0.5;
            let end_point = Point::new(0.0, 10.0);
            let end_angle = Angle::zero();

            let expected_result_rsl = RouteCSC {
                start: CirclePath {
                    center: Point::new(0.5, 0.0),
                    radius: radius,
                    angle: Angle::zero(),
                },
                tangent: StraightPath {
                    origin: Point::new(0.0, 0.0),
                    vector: Vector::from_angle_and_length(Angle::frac_pi_2(), 10.0),
                },
                end: CirclePath {
                    center: Point::new(-0.5, 10.0),
                    radius: radius,
                    angle: Angle::zero(),
                },
            };

            let result_rsl = RouteCSC::rsl(radius, end_point, end_angle).unwrap();

            assert!(result_rsl.start.approx_eq(expected_result_rsl.start));
            assert!(result_rsl.tangent.approx_eq(expected_result_rsl.tangent));
            assert!(result_rsl.end.approx_eq(expected_result_rsl.end));
        }
    }

    #[test]
    fn test_points_lsl() {
        {
            let radius = 0.5;
            let end_point = Point::new(0.0, 10.0);
            let end_angle = Angle::zero();

            let expected_result_lsl = RouteCSC {
                start: CirclePath {
                    center: Point::new(-0.5, 0.0),
                    radius: radius,
                    angle: Angle::zero(),
                },
                tangent: StraightPath {
                    origin: Point::new(0.0, 0.0),
                    vector: Vector::from_angle_and_length(Angle::frac_pi_2(), 10.0),
                },
                end: CirclePath {
                    center: Point::new(-0.5, 10.0),
                    radius: radius,
                    angle: Angle::zero(),
                },
            };

            let result_lsl = RouteCSC::lsl(radius, end_point, end_angle).unwrap();

            assert!(result_lsl.start.approx_eq(expected_result_lsl.start));
            assert!(result_lsl.tangent.approx_eq(expected_result_lsl.tangent));
            assert!(result_lsl.end.approx_eq(expected_result_lsl.end));
        }
    }

    #[test]
    fn test_point_lsr() {
        {
            let radius = 0.5;
            let end_point = Point::new(0.0, 10.0);
            let end_angle = Angle::zero();

            let expected_result_lsr = RouteCSC {
                start: CirclePath {
                    center: Point::new(-0.5, 0.0),
                    radius: radius,
                    angle: Angle::zero(),
                },
                tangent: StraightPath {
                    origin: Point::new(0.0, 0.0),
                    vector: Vector::from_angle_and_length(Angle::frac_pi_2(), 10.0),
                },
                end: CirclePath {
                    center: Point::new(0.5, 10.0),
                    radius: radius,
                    angle: Angle::zero(),
                },
            };

            let result_lsr = RouteCSC::lsr(radius, end_point, end_angle).unwrap();

            assert!(result_lsr.start.approx_eq(expected_result_lsr.start));
            assert!(result_lsr.tangent.approx_eq(expected_result_lsr.tangent));
            assert!(result_lsr.end.approx_eq(expected_result_lsr.end));
        }
    }

    #[test]
    fn test_point_rlr() {
        {
            let radius = 0.5;
            let end_point = Point::new(3.0, 0.0);
            let end_angle = Angle::pi();

            let expected_result_rlr = RouteCCC {
                start: CirclePath {
                    center: Point::new(0.5, 0.0),
                    radius: radius,
                    angle: Angle::pi(),
                },
                middle: CirclePath {
                    center: Point::new(1.5, 0.0),
                    radius: radius,
                    angle: Angle::pi(),
                },
                end: CirclePath {
                    center: Point::new(2.5, 0.0),
                    radius: radius,
                    angle: Angle::pi(),
                },
            };

            let result_rlr = RouteCCC::rlr(radius, end_point, end_angle).unwrap();

            assert!(result_rlr.start.approx_eq(expected_result_rlr.start));
            assert!(result_rlr.middle.approx_eq(expected_result_rlr.middle));
            assert!(result_rlr.end.approx_eq(expected_result_rlr.end));
        }
    }

    #[test]
    fn test_point_lrl() {
        {
            let radius = 0.5;
            let end_point = Point::new(-3.0, 0.0);
            let end_angle = Angle::pi();

            let expected_result_lrl = RouteCCC {
                start: CirclePath {
                    center: Point::new(-0.5, 0.0),
                    radius: radius,
                    angle: Angle::pi(),
                },
                middle: CirclePath {
                    center: Point::new(-1.5, 0.0),
                    radius: radius,
                    angle: Angle::pi(),
                },
                end: CirclePath {
                    center: Point::new(-2.5, 0.0),
                    radius: radius,
                    angle: Angle::pi(),
                },
            };

            let result_lrl = RouteCCC::lrl(radius, end_point, end_angle).unwrap();

            assert!(result_lrl.start.approx_eq(expected_result_lrl.start));
            assert!(result_lrl.middle.approx_eq(expected_result_lrl.middle));
            println!("{:?}", result_lrl.end);
            assert!(result_lrl.end.approx_eq(expected_result_lrl.end));
        }
    }
}
