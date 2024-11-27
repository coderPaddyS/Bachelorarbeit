use ba::{mesh::MeshNode, MyMesh};
use bevy::{app::{App, Startup}, prelude::Commands, render::{mesh::Indices, render_asset::RenderAssetUsages}, DefaultPlugins};
use bevy::prelude::*;


fn main() {


    App::new()
        .add_plugins(DefaultPlugins)
        .add_systems(Startup, setup)
        .add_systems(Update, input_handler)
        .run();
}


// System to receive input from the user,
// check out examples/input/ for more examples about user input.
fn input_handler(
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mesh_query: Query<&Handle<Mesh>, ()>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut query: Query<&mut Transform, ()>,
    time: Res<Time>,
) {
    if keyboard_input.pressed(KeyCode::KeyX) {
        for mut transform in &mut query {
            transform.rotate_x(time.delta_seconds() / 1.2);
        }
    }
    if keyboard_input.pressed(KeyCode::KeyY) {
        for mut transform in &mut query {
            transform.rotate_y(time.delta_seconds() / 1.2);
        }
    }
    if keyboard_input.pressed(KeyCode::KeyZ) {
        for mut transform in &mut query {
            transform.rotate_z(time.delta_seconds() / 1.2);
        }
    }
    if keyboard_input.pressed(KeyCode::KeyR) {
        for mut transform in &mut query {
            transform.look_to(Vec3::NEG_Z, Vec3::Y);
        }
    }
}


fn setup(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    
    let mut mesh = MyMesh::default();
    
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
        mesh.add_node(MeshNode::new(node))
    }).collect();
    println!("{nodes:#?}");
    vec![
        [0, 1, 2], [2, 1, 3],
        [1, 0, 5], [5, 0, 4],
        [0, 2, 4], [2, 6, 4],
        [4, 6, 5], [5, 6, 7],
        [2, 3, 6], [3, 7, 6],
        [3, 1, 7], [1, 5, 7]
    ].into_iter().for_each(|[a,b,c]| { mesh.add_triangle_by_nodes(nodes[a], nodes[b], nodes[c]).unwrap(); });

    commands.spawn(PbrBundle { mesh: meshes.add(mesh.build()), ..Default::default()} );

        // Transform for the camera and lighting, looking at (0,0,0) (the position of the mesh).
        let camera_and_light_transform =
        Transform::from_xyz(3.8, 3.8, 3.8).looking_at(Vec3::ZERO, Vec3::Y);

    // Camera in 3D space.
    commands.spawn(Camera3dBundle {
        transform: camera_and_light_transform,
        ..default()
    });

    // Light up the scene.
    commands.spawn(PointLightBundle {
        transform: camera_and_light_transform,
        ..default()
    });

}