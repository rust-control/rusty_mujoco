use rusty_mujoco::{mj_loadXML, mj_makeData, mj_step, mjr_render, mjv_updateScene};
use rusty_mujoco::{MjrContext, mjrRect, MjvScene, mjvCamera, mjvOption, mjtCatBit, mjtFontScale};

struct Args {
    xml_path: String,
    camera_name: Option<String>,
}
impl Args {
    fn from_env() -> Self {
        const USAGE: &str = "\
            Usage: visualize_left_object <path_to_xml_file> [options]\n\
            \n\
            Options:\n\
            \t--camera <camera name>\tSpecify the name of camera to use for rendering (optional)\n\
        ";
        
        let mut args = std::env::args().skip(1);
        let xml_path = args.next().expect(USAGE);
        
        let (mut camera_name,) = (None,);
        while let Some(arg) = args.next() {
            match &*arg {
                "--camera" => {
                    camera_name = Some(args.next().expect("Expected a camera name after --camera"));
                }
                _ => {
                    eprintln!("Unknown argument: {arg}");
                    eprintln!("{}", USAGE);
                    std::process::exit(1);
                }
            }
        }
        
        Self { xml_path, camera_name }
    }
}

fn main() {
    let Args { xml_path, camera_name } = Args::from_env();
    
    let model = mj_loadXML(xml_path).expect("Failed to load XML file");
    let mut data = mj_makeData(&model);
    
    let mut glfw = glfw::init(glfw::fail_on_errors).expect("Failed to initialize GLFW");
    let (mut window, events) = glfw
        .create_window(1200, 900, "Acrobot Simulation", glfw::WindowMode::Windowed)
        .expect("Failed to create GLFW window");
    window.set_size_polling(true);
    glfw::Context::make_current(&mut *window);

    let con = MjrContext::new(&model, mjtFontScale::X150);
    let opt = mjvOption::default();
    let mut scn = MjvScene::new(&model, 2000);
    let mut cam = mjvCamera::default();
    camera_name.map(|name| cam.set_fixedcamid(model.object_id(&name).expect("No camera of such name in the model")));
    
     while !window.should_close() {
         while data.time() < glfw.get_time() {
             mj_step(&model, &mut data);
         }
         
         let viewport = {
             let (width, height) = window.get_framebuffer_size();
             mjrRect::new(0, 0, width as u32, height as u32)
         };
         
        mjv_updateScene(
            &model,
            &mut data,
            &opt,
            None, /* No perturbation */
            &mut cam,
            mjtCatBit::ALL,
            &mut scn,
        );
        mjr_render(viewport, &mut scn, &con);
        glfw::Context::swap_buffers(&mut *window);
        
        glfw.poll_events();
        for (_, event) in glfw::flush_messages(&events) {
            match event {
                glfw::WindowEvent::Close => {
                    window.set_should_close(true);
                }
                glfw::WindowEvent::Size(width, height) => {
                    window.set_size(width, height);
                }
                _ => (),
            }
        }
     }
}
