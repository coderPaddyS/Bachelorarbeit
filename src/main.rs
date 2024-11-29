use std::f32::consts::PI;

use ba::{mesh::ClosedTriangleMeshNode, ClosedTriangleMesh};
use bevy::{app::{App, Startup}, prelude::Commands, render::{render_asset::RenderAssetUsages, render_resource::{Extent3d, TextureDimension, TextureFormat}}, DefaultPlugins};
use bevy::prelude::*;
use bevy::prelude::Mesh as BMesh;


fn main() {

    pretty_env_logger::init();

    App::new()
        .add_plugins(DefaultPlugins.set(ImagePlugin::default_nearest()))
        .add_systems(Startup, setup)
        .add_systems(Update, input_handler)
        .add_systems(Update, rotate)
        .run();
}

fn rotate(mut query: Query<&mut Transform, With<Shape>>, time: Res<Time>) {
    for mut transform in &mut query {
        transform.rotate_y(time.delta_seconds() / 2.);
    }
}


// System to receive input from the user,
// check out examples/input/ for more examples about user input.
fn input_handler(
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mesh_query: Query<&Handle<BMesh>, ()>,
    meshes: ResMut<Assets<BMesh>>,
    mut query: Query<&mut Transform, With<Shape>>,
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

#[derive(Component)]
struct Shape;

fn setup(
    mut commands: Commands,
    mut images: ResMut<Assets<Image>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<bevy::prelude::Mesh>>,
) {
    
    let mut mesh = ClosedTriangleMesh::default();
    
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
        mesh.add_node(ClosedTriangleMeshNode::new(node))
    }).collect();
    vec![
        [2, 1, 0], [3, 1, 2],
        [5, 0, 1], [4, 0, 5],
        [4, 2, 0], [4, 6, 2],
        [5, 6, 4], [7, 6, 5],
        [6, 3, 2], [6, 7, 3],
        [7, 1, 3], [7, 5, 1]
    ].into_iter().for_each(|[a,b,c]| { mesh.add_triangle_by_nodes(nodes[a], nodes[b], nodes[c]).unwrap(); });

    let debug_material = materials.add(StandardMaterial {
        base_color_texture: Some(images.add(uv_debug_texture())),
        ..default()
    });

    const SHAPES_X_EXTENT: f32 = 14.0;
    const EXTRUSION_X_EXTENT: f32 = 16.0;
    const Z_EXTENT: f32 = 5.0;

    for triangle in mesh.build_many() {
        commands.spawn((PbrBundle { 
            mesh: meshes.add(triangle), 
            material: debug_material.clone(),
            // transform: Transform::from_rotation(Quat::from_rotation_x(-PI / 4.)),
    
            ..Default::default()
        }, Shape));
    }

    // commands.spawn((PbrBundle { 
    //     mesh: meshes.add(mesh.build()), 
    //     material: debug_material.clone(),
    //     // transform: Transform::from_rotation(Quat::from_rotation_x(-PI / 4.)),

    //     ..Default::default()
    // }, Shape));

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