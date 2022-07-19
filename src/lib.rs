pub mod fbot_rust_client {
    use std::io::Cursor;
    use prost::Message;
    use std::net::UdpSocket;

    mod fira_protos;
    
    const VISION_ADDRS: &str = "224.0.0.1:10002";
    const COMMAND_ADDRS: &str = "127.0.0.1:20011";

    const ORIENTATION_KP: f64 = 10.0;
    const ROBOT_SPEED: f64 = 20.0;

    #[derive(Debug)]
    pub enum Team{
        Yellow,
        Blue
    }

    pub fn serialize_packet(packet: &fira_protos::Packet) -> Vec<u8> {
        let mut buf = Vec::new();
        buf.reserve(packet.encoded_len());
        
        // Unwrap is safe, since we have reserved sufficient capacity in the vector.
        packet.encode(&mut buf).unwrap();
        buf
    }

    pub fn deserialize_env(buf: &[u8]) -> Result<fira_protos::Environment, prost::DecodeError> {
        fira_protos::Environment::decode(&mut Cursor::new(buf))
    }

    pub fn send_command(commands: fira_protos::Commands) {
        let socket_sender = UdpSocket::bind(VISION_ADDRS).unwrap();

        let packet = fira_protos::Packet {
            cmd: Some(commands),
            replace: None        
        };
        let buf = serialize_packet(&packet); 

        match socket_sender.send_to(&buf, COMMAND_ADDRS) {
            Ok(_) => {},
            Err(e) => {
                println!("Error Send {}", e)
            }
        };
    }

    fn get_frame() -> Option<fira_protos::Frame>{
        let socket = UdpSocket::bind(VISION_ADDRS).unwrap();
        let mut buf = [0; 1024];

        let (len, addr) = socket.recv_from(&mut buf).unwrap();

        let env = deserialize_env(&buf[..len]).unwrap();

        env.frame
    }

    pub fn get_ball() -> fira_protos::Ball {
        let mut ret = fira_protos::Ball{
            x: 0.0,
            y: 0.0,
            z: 0.0,
            vx: 0.0,
            vy: 0.0,
            vz: 0.0,
        };

        if let Some(frame) = get_frame() {
            if let Some(ball) = frame.ball {
                ret = ball
            }
        }

        ret
    }

    pub fn get_blue_robot(id: &u32) -> Option<fira_protos::Robot> {
        if let Some(frame) = get_frame() {
            let mut ret = None;

            for robot in frame.robots_blue {
                if robot.robot_id == *id {
                    ret = Some(robot)
                }
            };

            ret

        } else { None }
    }

    pub fn get_yellow_robot(id: &u32) -> Option<fira_protos::Robot> {
        if let Some(frame) = get_frame() {
            let mut ret = None;

            for robot in frame.robots_yellow {
                if robot.robot_id == *id {
                    ret = Some(robot)
                }
            };

            ret

        } else { 
            None 
        }
    }
    
    #[derive(Debug)]
    struct Point {
        x: f64,
        y: f64
    }
    
    pub fn get_ball_cp() -> (f64, f64){
        let ball = get_ball();
        let ball_point = Point::new(ball.x, ball.y);

        let GOAL_POINT = Point::new(-0.75, 0.0);
        let CP = 0.5;
        
        let orientation_to_goal = ball_point.orientation_to(&GOAL_POINT);
        
        let cp_x = orientation_to_goal.cos() * CP;
        let cp_y = orientation_to_goal.sin() * CP;

        (ball.x + cp_x, ball.y + cp_y)
    }

    impl Point {
        pub fn new(x: f64, y: f64) -> Self {
            Self {
                x: x,
                y: y
            }
        }

        pub fn orientation_to(&self, p: &Point) -> f64 {
            let x = p.x - self.x;
            let y = p.y - self.y;

            y.atan2(x)
        }

        pub fn distance_to(&self, p: &Point) -> f64 {
            let x = p.x - self.x;
            let y = p.y - self.y;

            (x*x + y*y).sqrt()
        }
    }

    #[derive(Debug)]
    pub struct Robot {
        id: u32,
        team: Team,
    }

    impl Robot {
        pub fn new(id: u32, team: Team) -> Self {
            Self {
                id: id,
                team: team,
            }
         }

        fn get_robot(&self) -> fira_protos::Robot{
            match self.team {
                Team::Yellow => get_yellow_robot(&self.id).unwrap(),
                Team::Blue => get_blue_robot(&self.id).unwrap()
            }
        }

        pub fn get_x(&self) -> f64 {
            self.get_robot().x
        }

        pub fn get_y(&self) -> f64 {
            self.get_robot().y
        }

        pub fn get_orientation(&self) -> f64 {
            self.get_robot().orientation
        }

        pub fn get_control_point(&self) -> (f64, f64) {
            let CP = 0.1;
            let (x, y, orientation) = (self.get_x(), self.get_y(), self.get_orientation());

            let cp_x = orientation.cos() * CP;
            let cp_y = orientation.sin() * CP;

            (x + cp_x, y + cp_y)
        }

        pub fn set_speed(&self, wheel_left: f64, wheel_right: f64) {
            let commands = fira_protos::Commands {
                robot_commands: vec![
                    fira_protos::Command {
                        id: 0,
                        yellowteam: true,
                        wheel_left: wheel_left,
                        wheel_right: wheel_right,
                    },
                ]
            };

            send_command(commands);
        }

        pub fn go_to(&self, target_x: f64, target_y:f64) {
            loop {
                let diff_x = target_x - self.get_x();
                let diff_y = target_y - self.get_y();

                let dist = (diff_x*diff_x + diff_y*diff_y).sqrt();

                if dist < 0.08 {
                    break;
                }

                let mut goalie_orientation = self.get_orientation();
                let target_orientation = (diff_y/diff_x).atan();

                // ele tem problema ao calcular a diferença entre os quadrantes 1 2 e 3 4, ele não rataciona pelo menor caminho
                
                if diff_x < 0.0 {
                    if goalie_orientation > 0.0 {
                        goalie_orientation -= std::f64::consts::PI;
                    } else {
                        goalie_orientation += std::f64::consts::PI;
                    }
                }

                let err = target_orientation - goalie_orientation;
                let velocidade = err * ORIENTATION_KP;
                
                self.set_speed(-velocidade + ROBOT_SPEED, velocidade + ROBOT_SPEED);
            };

            // self.set_speed(0.0, 0.0);

        }
    }
}

