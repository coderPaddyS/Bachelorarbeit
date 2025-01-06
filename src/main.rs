use std::{f32::consts::PI, ops::{Deref, DerefMut}};

use ba::{mesh::{FromMeshBuilder, Node, UnfinishedNode}, ClosedTriangleMesh};
use bevy::{app::{App, Startup}, ecs::query::QueryData, input::{keyboard::KeyboardInput, mouse::MouseMotion, ButtonState}, prelude::Commands, render::{mesh::Indices, render_asset::RenderAssetUsages, render_resource::{Extent3d, PipelineDescriptor, RenderPipelineDescriptor, TextureDimension, TextureFormat}, view::WindowSurfaces}, window::{CursorGrabMode, PrimaryWindow}, DefaultPlugins};
use bevy::prelude::*;
use bevy::prelude::Mesh as BMesh;
use csv::StringRecord;
use nalgebra::coordinates;


#[derive(Component)]
struct OrbifoldMesh(ClosedTriangleMesh);

#[derive(Component)]
struct CameraRotation {
    yaw: f32,
    pitch: f32
}

impl Deref for OrbifoldMesh {
    type Target = ClosedTriangleMesh;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl DerefMut for OrbifoldMesh {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl OrbifoldMesh {
    pub fn build_mesh(&self) -> bevy::prelude::Mesh {
        let (colors, nodes): (Vec<Vec4>, Vec<[f32; 3]>) = self.triangles
            .iter()
            .cloned()
            .filter_map(|tri| {
                match tri {
                    None => None,
                    Some(tri) => {
                        let color = Color::srgba_u8(rand::random(), rand::random(), rand::random(), 128).to_srgba().to_vec4();
                        let nodes: Vec<_> = tri.corners.iter().map(|idx| self[*idx].as_ref().unwrap().coordinates.clone()).collect();
                        // let normal = (Vector3::from(nodes[1]) - Vector3::from(nodes[0])).cross(&(Vector3::from(nodes[2]) - Vector3::from(nodes[0])));

                        Some((color, nodes))
                    }
                }
            })
            .map(|(color, nodes)| {
                nodes.into_iter().map(move |node| (color, node))
            })
            .flatten()
            .unzip();

        bevy::prelude::Mesh::new(
                bevy::render::mesh::PrimitiveTopology::TriangleList,
                RenderAssetUsages::default()
            )
            .with_inserted_indices(Indices::U32((0..nodes.len()).map(|i| i as u32).collect::<Vec<_>>()))
            .with_inserted_attribute(bevy::prelude::Mesh::ATTRIBUTE_POSITION, nodes)
            .with_inserted_attribute(bevy::prelude::Mesh::ATTRIBUTE_COLOR, colors)
            // .apply(|(colors, other): (Vec<Vec4>, Vec<([f32; 3], Vector3<f32>)>)| (colors, other.unzip()));
    }
}

fn main() {

    pretty_env_logger::init();

    App::new()
        .add_plugins(DefaultPlugins.set(ImagePlugin::default_nearest()))
        .add_systems(Startup, setup)
        .add_systems(Update, (input_handler, mouse_rotation, move_camera, collapse_edge).chain())
        // .add_systems(Update, rotate)
        .run();
}

fn rotate(mut query: Query<&mut Transform, With<OrbifoldMesh>>, time: Res<Time>) {
    for mut transform in &mut query {
        transform.rotate_y(time.delta_secs() / 2.);
    }
}


// System to receive input from the user,
// check out examples/input/ for more examples about user input.
fn input_handler(
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut query: Query<&mut Transform, With<OrbifoldMesh>>,
    time: Res<Time>,
) {
    if keyboard_input.pressed(KeyCode::KeyX) {
        for mut transform in &mut query {
            transform.rotate_x(time.delta_secs() / 1.2);
        }
    }
    if keyboard_input.pressed(KeyCode::KeyY) {
        for mut transform in &mut query {
            transform.rotate_y(time.delta_secs() / 1.2);
        }
    }
    if keyboard_input.pressed(KeyCode::KeyZ) {
        for mut transform in &mut query {
            transform.rotate_z(time.delta_secs() / 1.2);
        }
    }
    if keyboard_input.pressed(KeyCode::KeyR) {
        for mut transform in &mut query {
            transform.look_to(Vec3::NEG_Z, Vec3::Y);
        }
    }
}

fn move_camera(
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut query: Single<&mut Transform, With<Camera>>,
    time: Res<Time>
) {
    let mut transform = query.into_inner();
    let mut direction = Vec3::ZERO;

    if keyboard_input.pressed(KeyCode::KeyW) || keyboard_input.pressed(KeyCode::ArrowUp) {
        direction += *transform.forward(); // Move forward
    }
    if keyboard_input.pressed(KeyCode::KeyS) || keyboard_input.pressed(KeyCode::ArrowDown) {
        direction += *transform.back(); // Move backward
    }
    if keyboard_input.pressed(KeyCode::KeyA) || keyboard_input.pressed(KeyCode::ArrowLeft) {
        direction += *transform.left(); // Move left
    }
    if keyboard_input.pressed(KeyCode::KeyD) || keyboard_input.pressed(KeyCode::ArrowRight) {
        direction += *transform.right(); // Move right
    }
    if keyboard_input.pressed(KeyCode::ShiftLeft) {
        direction += *transform.down(); // Move left
    }
    if keyboard_input.pressed(KeyCode::Space) {
        direction += *transform.up(); // Move right
    }

    if direction.length() > 0.0 {
        direction = direction.normalize() * 0.2;
        transform.translation += direction; // Update the camera's position
    }
}

fn mouse_rotation(
    mut mouse_motion_events: EventReader<MouseMotion>,
    query: Single<(&mut Transform, &mut CameraRotation), With<Camera>>,
) {
    let (mut transform, mut rotation) = query.into_inner();
    for event in mouse_motion_events.read() {
        // Adjust the rotation based on mouse movement
        let sensitivity = 0.1; // Adjust sensitivity as needed
        rotation.yaw -= event.delta.y * sensitivity;
        rotation.pitch -= event.delta.x * sensitivity;

        // Apply rotation
        // Quat::from_rotation_y(yaw) + Quat::from_rotation_x(angle)
        transform.rotation = Quat::from_euler(EulerRot::ZYX, rotation.yaw.to_radians(), rotation.pitch.to_radians(), 0.0);
    }
}

fn collapse_edge(
    mut keyboard_input: EventReader<KeyboardInput>,
    mut meshes: ResMut<Assets<bevy::prelude::Mesh>>,
    mesh: Single<(&mut Mesh3d, &mut OrbifoldMesh)>,
) {
    let (mut mesh, mut orbifold) = mesh.into_inner();
    for input in keyboard_input.read() {
        if input.key_code == KeyCode::KeyC && input.state  == ButtonState::Released {
            println!("contracting!");
            orbifold.contract_next_edge();
            mesh.0 = meshes.add(orbifold.build_mesh())
        }
        if input.key_code == KeyCode::KeyU && input.state  == ButtonState::Released {
            println!("uncontracting!");
            orbifold.uncontract_next_edge();
            mesh.0 = meshes.add(orbifold.build_mesh())
        }
    }
}

fn setup(
    mut commands: Commands,
    mut images: ResMut<Assets<Image>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<bevy::prelude::Mesh>>,
    mut window: Single<&mut Window, With<PrimaryWindow>>
) {
    
    let mut builder = ba::mesh::MeshBuilder::default();
    let nodes: Vec<_> = vec![
        [1.0f32,1.0f32,1.0f32],
        [1.0f32,1.0f32,-1.0f32],
        [1.0f32,-1.0f32,1.0f32],
        [1.0f32,-1.0f32,-1.0f32],
        [-1.0f32,1.0f32,1.0f32],
        [-1.0f32,1.0f32,-1.0f32],
        [-1.0f32,-1.0f32,1.0f32],
        [-1.0f32,-1.0f32,-1.0f32],
    ].into_iter().map(|node| {
        builder.add_node(UnfinishedNode::new(node))
    }).collect();
    vec![
        [2, 0, 1], [3,2, 1],
        [5, 1, 0], [4,5, 0],
        [4, 0, 2], [4,2, 6],
        [5, 4, 6], [7,5, 6],
        [6, 2, 3], [6,3, 7],
        [7, 3, 1], [7,1, 5]
    ].into_iter()
        .for_each(|[a,b,c]| { 
            builder.add_triangle_by_nodes(nodes[a], nodes[b], nodes[c]).unwrap();
        });
    
    debug!("building mesh");
    let mesh = OrbifoldMesh(ClosedTriangleMesh::build(builder).unwrap());

    let debug_material = materials.add(StandardMaterial {
        base_color_texture: Some(images.add(uv_debug_texture())),
        cull_mode: None,
        ..default()
    });

    commands.spawn((
        Mesh3d(meshes.add(mesh.build_mesh())),
        MeshMaterial3d(debug_material), 
        mesh
    ));

        let camera_and_light_transform =
        Transform::from_xyz(3.8, 3.8, 3.8).looking_at(Vec3::ZERO, Vec3::Y);

    commands.spawn((Camera3d::default(), camera_and_light_transform, CameraRotation { yaw: 0.0, pitch: 0.0 }));
    commands.spawn((PointLight::default(), camera_and_light_transform));

    window.cursor_options.grab_mode = CursorGrabMode::Locked; 
    window.cursor_options.visible = false;

    /// Creates a colorful test pattern
fn uv_debug_texture() -> Image {
    const TEXTURE_SIZE: usize = 8;

    let mut palette: [u8; 32] = [
        255, 102, 159, 255, 255, 159, 102, 255, 236, 255, 102, 255, 121, 255, 102, 255, 102, 255,
        198, 255, 102, 198, 255, 255, 121, 102, 255, 255, 236, 102, 255, 255,
    ];

    let mut texture_data = [0; TEXTURE_SIZE * TEXTURE_SIZE * 4];
    for y in 0..TEXTURE_SIZE {
        let offset = TEXTURE_SIZE * y * 4;
        texture_data[offset..(offset + TEXTURE_SIZE * 4)].copy_from_slice(&palette);
        palette.rotate_right(4);
    }

    Image::new_fill(
        Extent3d {
            width: TEXTURE_SIZE as u32,
            height: TEXTURE_SIZE as u32,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        &texture_data,
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::RENDER_WORLD,
    )
}

}