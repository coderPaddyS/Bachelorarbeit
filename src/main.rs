#![feature(iter_array_chunks)]
#![feature(trivial_bounds)]

use std::{any::Any, f32::consts::PI, ffi::OsStr, ops::{Deref, DerefMut}, path::{Path, PathBuf}};

use apply::Apply;
use ba::{mesh::{ContractableMesh, Edge, FromMeshBuilder, Index, List, MeshBuilder, MeshData, MeshTriangleInfo, Node, TriangleMesh, UncontractableMesh, UnfinishedNode}, projective_structure::{self, structure::{self, ProjectiveStructure}, visualisation::{ProjectiveStructureVisualisation, VisualiseProjectiveStructure}, CEquationParameters, CEquations, CalculateProjectiveStructure, CalculationProjectiveStructureStepsize, SubdividedTriangleData, SubdiviveMesh}, ClosedTriangleMesh};
use bevy::{app::{App, Startup}, ecs::query::QueryData, input::{keyboard::KeyboardInput, mouse::MouseMotion, ButtonState}, prelude::Commands, render::{mesh::Indices, render_asset::RenderAssetUsages, render_resource::{Extent3d, PipelineDescriptor, RenderPipelineDescriptor, TextureDimension, TextureFormat}, view::WindowSurfaces}, utils::hashbrown::HashMap, window::{CursorGrabMode, PrimaryWindow}, DefaultPlugins};
use bevy::prelude::*;
use bevy::prelude::Mesh as BMesh;
use bevy_egui::{EguiContext, EguiContexts, EguiPlugin};
use csv::StringRecord;
use nalgebra::coordinates;

enum InnerType {
    Mesh(ClosedTriangleMesh),
    ProjectiveStructure(ProjectiveStructure<ClosedTriangleMesh>),
    ControlNetProjectiveStructure {
        structure: ProjectiveStructure<ClosedTriangleMesh<SubdividedTriangleData>>,
        mapping: Vec<(Index<Edge>, Index<Edge>)>,
        history: Vec<CEquationParameters>,
        current: usize
    },
    UncontractableProjectiveStructure(ProjectiveStructure<ClosedTriangleMesh<SubdividedTriangleData>>)
}

impl InnerType {
    fn build_mesh(&self) -> MeshTriangleInfo {
        match self {
            InnerType::Mesh(mesh) => mesh.build_mesh(),
            InnerType::ProjectiveStructure(structure) => structure.build_mesh(),
            InnerType::ControlNetProjectiveStructure { structure, .. } => structure.build_mesh(),
            InnerType::UncontractableProjectiveStructure(structure) => structure.build_mesh()
        }
    }

    fn uncontract_edge(&mut self, info: <<ClosedTriangleMesh as TriangleMesh>::Data as MeshData>::CollapseInfoOutput) {
        match self {
            InnerType::Mesh(mesh) => mesh.uncontract_edge(info),
            InnerType::ProjectiveStructure(structure) => structure.uncontract_edge(info),
            InnerType::ControlNetProjectiveStructure { structure, .. } => structure.uncontract_edge(info),
            InnerType::UncontractableProjectiveStructure(structure) => structure.uncontract_edge(info)
        }
    }

    fn uncontract_next_edge(&mut self) {
        match self {
            InnerType::Mesh(mesh) => mesh.uncontract_next_edge(|_, _| {}),
            InnerType::ProjectiveStructure(structure) => structure.uncontract_next_edge(|_, _| {}),
            InnerType::ControlNetProjectiveStructure { structure , ..} => structure.uncontract_next_edge(|_, _| {}),
            InnerType::UncontractableProjectiveStructure(structure) => structure.uncontract_next_edge(|_, _| {})
        }
    }
}

#[derive(Component)]
struct OrbifoldMesh(InnerType);

#[derive(Component)]
struct CameraRotation {
    yaw: f32,
    pitch: f32
}

impl Deref for OrbifoldMesh {
    type Target = InnerType;
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
        // let MeshTriangleInfo { triangles, nodes } = self.0.clone().subdivide().build_mesh();
        let MeshTriangleInfo { triangles, nodes } = self.0.build_mesh();
        // println!("{:?}", self.0.contraction_order);
        // let MeshTriangleInfo { triangles, nodes } = self.0.clone().build_mesh();
        let (indices, colors): (Vec<u32>, Vec<Vec4>) = triangles.into_iter().map(|(i, [r,g,b,a])| (i as u32, Color::srgba_u8(r,g,b,a).to_srgba().to_vec4())).unzip();

        bevy::prelude::Mesh::new(
                bevy::render::mesh::PrimitiveTopology::TriangleList,
                RenderAssetUsages::default()
            )
            .with_inserted_indices(Indices::U32(indices))
            .with_inserted_attribute(bevy::prelude::Mesh::ATTRIBUTE_POSITION, nodes)
            .with_inserted_attribute(bevy::prelude::Mesh::ATTRIBUTE_COLOR, colors)
            // .apply(|(colors, other): (Vec<Vec4>, Vec<([f32; 3], Vector3<f32>)>)| (colors, other.unzip()));
    }
}

#[derive(Default)]
struct FileDialog {
    pub dialog: Option<egui_file::FileDialog>
}

impl FileDialog {
    
    fn openMesh() -> impl FnMut(
            Res<ButtonInput<KeyCode>>,
            EguiContexts,
            ResMut<Assets<bevy::prelude::Mesh>>,
            Single<(&mut Mesh3d, &mut OrbifoldMesh)>
        ) -> ()
    {
        let mut file_dialog = Self { dialog: None };
        return move |input_handler, mut contexts, mut meshes, mut query| {
            if let Some(dialog) = &mut file_dialog.dialog {
                dialog.show(contexts.ctx_mut());
                if dialog.selected() {
                    let files = dialog.selection();
                    if files.len() == 2 {
                        let ver = files.iter().find(|path| path.extension() == Some(OsStr::new("ver"))).cloned();
                        let tri = files.iter().find(|path| path.extension() == Some(OsStr::new("tri"))).cloned();
                        if let (Some(ver), Some(tri)) = (ver, tri) {
                            let mut builder = MeshBuilder::default();
                            let nodes: Vec<_> = csv::ReaderBuilder::new()
                                .has_headers(false)
                                .from_path(ver)
                                .unwrap()
                                .records()
                                .enumerate()
                                .map(| (i, result)| {
                                    println!("line: {i}");
                                    let coordinates: [f32; 3] = result.unwrap()
                                        .into_iter()
                                        .enumerate()
                                        .map(|(index, c)| c.parse::<f32>().expect(&format!("Error paring index: {index}, c: {c}"))).collect::<Vec<_>>().try_into().unwrap();
                                    println!("{coordinates:?}");
                                    builder.add_node(UnfinishedNode::new(coordinates))
                                })
                                .collect();
                    
                            println!("{}, {nodes:?}", nodes.len());
                            let triangles: Vec<_> = csv::ReaderBuilder::new()
                                .has_headers(false)
                                .from_path(tri)
                                .unwrap()
                                .records()
                                .enumerate()
                                .map(| (i, result)| {
                                    println!("line: {i}");
                                    let [a,b,c] = result.unwrap()
                                        .into_iter()
                                        .enumerate()
                                        .map(|(index, c)| c.parse::<usize>().expect(&format!("Error paring index: {index}, c: {c}")))
                                        .map(|index| (index - 1).into())
                                        .collect::<Vec<_>>().try_into().unwrap();
                                    debug!("a: {a}, b: {b}, c: {c}");
                                    builder.add_triangle_by_nodes(a, b, c).unwrap()
                                })
                                .collect();

                            let (mut mesh, mut orbifold) = query.into_inner();
                            orbifold.0 = InnerType::Mesh(ClosedTriangleMesh::build(builder).unwrap());
                            mesh.0 = meshes.add(orbifold.build_mesh());
                            file_dialog.dialog = None;
                        } else {
                            file_dialog.dialog = None;
                        }
                    } else if files.len() == 1 && files[0].extension() == Some(OsStr::new("norm")) { 
                        let mut reader = csv::ReaderBuilder::new()
                            .has_headers(false)
                            .delimiter(b' ')
                            .from_path(files[0])
                            .unwrap();
                        let mut records = reader.records();
                        let mut nodes = Vec::<UnfinishedNode>::new();
                        let triangles = records.array_chunks::<6>()
                            .map(|[a, _, b, _, c, _]| [a.unwrap(), b.unwrap(), c.unwrap()])
                            .map(|vertices| {
                                vertices
                                    .map(|node| {
                                        println!("record: {node:?}");
                                        let n: [f64;3] = node.into_iter()
                                            .map(|c| c.parse::<f64>().unwrap())
                                            .collect::<Vec<_>>()
                                            .try_into()
                                            .unwrap();
                                        n
                                    })
                            })
                            .collect::<Vec<_>>();
                        let triangles= triangles.into_iter()
                            .map(|vertices| {
                                vertices.map(|coordinates| {
                                    let index = nodes.iter()
                                        .enumerate()
                                        .find(|(_, node)| node.coordinates == coordinates)
                                        .map(|(index, _)| index)
                                        .unwrap_or_else(|| {
                                            nodes.push(UnfinishedNode::new_f64(coordinates));
                                            nodes.len() - 1
                                        });
                                    Into::<Index<UnfinishedNode>>::into(index)
                                })
                            })
                            .collect::<Vec<_>>();
                        let mut builder = MeshBuilder::default();
                        nodes.into_iter().for_each(|node| { builder.add_node(node); });
                        triangles.into_iter().for_each(|[a,b,c]| { builder.add_triangle_by_nodes(a, b, c).unwrap(); });
                        let (mut mesh, mut orbifold) = query.into_inner();
                        println!("{:?}", builder.nodes.len());
                        println!("{:?}", builder.edges.iter().filter(|edge| edge.as_ref().unwrap().previous.is_none()).count());
                        orbifold.0 = InnerType::Mesh(ClosedTriangleMesh::build(builder).unwrap());
                        mesh.0 = meshes.add(orbifold.build_mesh());
                        file_dialog.dialog = None;
                    } else {
                        file_dialog.dialog = None;
                    }
                }
            } else if input_handler.just_pressed(KeyCode::KeyO) {
                println!("open file");
                let file_filter = Box::new({
                    let extensions = [Some(OsStr::new("tri")), Some(OsStr::new("ver")), Some(OsStr::new("norm"))];
                    move |path: &Path| -> bool { extensions.contains(&path.extension())}
                });
                let mut dialog = egui_file::FileDialog::open_file(None).show_files_filter(file_filter).multi_select(true);
                dialog.open();
                file_dialog.dialog = Some(dialog);
            }
        }
    }
}
fn main() {
    let file_dialog = egui_file::FileDialog::open_file(None);
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(EguiPlugin)
        .add_systems(Startup, setup)
        .add_systems(Update,  FileDialog::openMesh())
        .add_systems(Update, (input_handler, mouse_rotation, move_camera, collapse_edge))
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
        rotation.pitch += event.delta.x * sensitivity;
        rotation.yaw += event.delta.y * sensitivity;

        // Apply rotation
        // Quat::from_rotation_y(yaw) + Quat::from_rotation_x(angle)
        transform.rotation = Quat::from_euler(EulerRot::YXZ, rotation.pitch.to_radians(), rotation.yaw.to_radians(), 0.0);
    }
}

fn to_bevy_mesh(MeshTriangleInfo { triangles, nodes }: MeshTriangleInfo) -> bevy::prelude::Mesh {
    let (indices, colors): (Vec<u32>, Vec<Vec4>) = triangles.into_iter().map(|(i, [r,g,b,a])| (i as u32, Color::srgba_u8(r,g,b,a).to_srgba().to_vec4())).unzip();

    bevy::prelude::Mesh::new(
            bevy::render::mesh::PrimitiveTopology::TriangleList,
            RenderAssetUsages::default()
        )
        .with_inserted_indices(Indices::U32(indices))
        .with_inserted_attribute(bevy::prelude::Mesh::ATTRIBUTE_POSITION, nodes)
        .with_inserted_attribute(bevy::prelude::Mesh::ATTRIBUTE_COLOR, colors)
}

fn collapse_edge(
    mut keyboard_input: EventReader<KeyboardInput>,
    mut meshes: ResMut<Assets<bevy::prelude::Mesh>>,
    mesh: Single<(&mut Mesh3d, &mut OrbifoldMesh)>,
) {
    let (mut mesh, mut orbifold) = mesh.into_inner();
    for input in keyboard_input.read() {
        if input.key_code == KeyCode::KeyC && input.state  == ButtonState::Released {
            if let InnerType::Mesh(orbmesh) = &mut orbifold.0 {
                println!("contracting!");
                orbmesh.contract_next_edge();                
                mesh.0 = meshes.add(orbifold.build_mesh())
            }
            // println!("contraction list: {:?}", orbifold.contraction_order.clone().into_sorted_iter().collect::<Vec<_>>());
        }
        if input.key_code == KeyCode::KeyM && input.state  == ButtonState::Released {
            if let InnerType::Mesh(orbmesh) = &mut orbifold.0 {
                println!("contracting max!");
                orbmesh.contract_max();                
                mesh.0 = meshes.add(orbifold.build_mesh())
            }
            // println!("contraction list: {:?}", orbifold.contraction_order.clone().into_sorted_iter().collect::<Vec<_>>());
        }
        if input.key_code == KeyCode::KeyU && input.state  == ButtonState::Released {
            println!("uncontracting!");
            orbifold.uncontract_next_edge();
            if let InnerType::ControlNetProjectiveStructure { mapping, structure, history, current } = &mut orbifold.0 {
                orbifold.0 = InnerType::UncontractableProjectiveStructure(structure.clone())
            }
            mesh.0 = meshes.add(orbifold.build_mesh());      
        }
        if input.key_code == KeyCode::KeyP && input.state  == ButtonState::Released {
            if let InnerType::Mesh(orbmesh) = &orbifold.0 {
                println!("projective structure!");
                // let m = orbmesh.clone().apply(|it| (it.clone(), it.calculate_projective_structure())).apply(|(mut mesh, structure)| {
                    // let mut coefficients = List::with_defaults(mesh.edges.len());
                    // structure.into_iter().for_each(|((index, _), coefficient)| coefficients[index] = Some(coefficient));
                    // ProjectiveStructure::from_bare(coefficients, mesh)
                // });
                let (mapping, structure, history) = orbmesh.clone()
                    .subdivide()
                    .calculate_projective_structure(1e-9, 5, 1, CalculationProjectiveStructureStepsize::Break(0.125f64))
                    .apply(|(s, h)| (s.mapping.clone(), ProjectiveStructure::new(s), h));
                
                ProjectiveStructureVisualisation::new(&structure).visualise().apply(|info| mesh.0 = meshes.add(to_bevy_mesh(info)));
                
                orbifold.0 = InnerType::ControlNetProjectiveStructure { mapping, structure, current: history.len() - 1, history };
            }
        }
        if input.key_code == KeyCode::Digit0 && input.state  == ButtonState::Released {
            if let InnerType::ControlNetProjectiveStructure { mapping, structure, history, current } = &mut orbifold.0 {
                *current = 0;
                structure.replace(history[0].clone(), mapping);
                ProjectiveStructureVisualisation::new(&structure).visualise().apply(|info| mesh.0 = meshes.add(to_bevy_mesh(info)));
            }
        }
        if input.key_code == KeyCode::Period && input.state  == ButtonState::Released {
            if let InnerType::ControlNetProjectiveStructure { mapping, structure, history, current } = &mut orbifold.0 {
                if *current < history.len() - 1 {
                    *current = *current + 1;
                    structure.replace(history[*current].clone(), mapping);
                    ProjectiveStructureVisualisation::new(&structure).visualise().apply(|info| mesh.0 = meshes.add(to_bevy_mesh(info)));
                }
            }
        }
        if input.key_code == KeyCode::Comma && input.state  == ButtonState::Released {
            if let InnerType::ControlNetProjectiveStructure { mapping, structure, history, current } = &mut orbifold.0 {
                if *current > 0 {
                    *current = *current - 1;
                    structure.replace(history[*current].clone(), mapping);
                    ProjectiveStructureVisualisation::new(&structure).visualise().apply(|info| mesh.0 = meshes.add(to_bevy_mesh(info)));
                }
            }
        }
        if input.key_code == KeyCode::KeyI && input.state  == ButtonState::Released {
            if let InnerType::Mesh(orbmesh) = &orbifold.0 {
                println!("saving mesh!");
                orbmesh.clone().save_to_dir(&mut PathBuf::from("./output/"), "orbifold");
            } else if let InnerType::ProjectiveStructure(structure) = &orbifold.0 {
                println!("saving projective structure!");
                structure.clone().save_to_dir(&mut PathBuf::from("./output/"), "orbifold");
            } else if let InnerType::ControlNetProjectiveStructure { structure, ..} = &orbifold.0 {
                println!("saving projective structure!");
                structure.clone().save_to_dir(&mut PathBuf::from("./output/"), "orbifold");
            }
        }
        if input.key_code == KeyCode::KeyE && input.state  == ButtonState::Released {
            println!("loading mesh!");
            orbifold.0 = InnerType::Mesh(ClosedTriangleMesh::restore_from_dir(&mut PathBuf::from("./output/"), "orbifold"));
            mesh.0 = meshes.add(orbifold.build_mesh())
        }
        if input.key_code == KeyCode::KeyN && input.state  == ButtonState::Released {
            println!("loading projective structure via mesh and coefficients!");
            // orbifold.0 = InnerType::ProjectiveStructure(ProjectiveStructure::restore_from_dir(&mut PathBuf::from("./output/"), "orbifold"));
            // orbifold.0 = InnerType::ProjectiveStructure(ProjectiveStructure::restore_from_dir(&mut PathBuf::from("./output/"), "orbifold", 4));
            orbifold.0 = InnerType::ProjectiveStructure(ProjectiveStructure::restore_from_dir(&mut PathBuf::from("./output/"), "orbifold"));
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
        [-1f32,1.0f32,1.0f32],
        [-1.0f32,1.0f32,-1.0f32],
        [-1.0f32,-1.0f32,1.0f32],
        [-1.0f32,-1.0f32,-1.0f32],
    ].into_iter().map(|node| {
        builder.add_node(UnfinishedNode::new(node))
    }).collect();
    vec![
        [0, 1, 2], [2, 1, 3],
        [1, 0, 5], [5, 0, 4],
        [0, 2, 4], [2, 6, 4],
        [4, 6, 5], [5, 6, 7],
        [2, 3, 6], [3, 7, 6],
        [3, 1, 7], [1, 5, 7]
    ].into_iter()
        .for_each(|[a,b,c]| { 
            builder.add_triangle_by_nodes(nodes[a], nodes[b], nodes[c]).unwrap();
        });
    
    debug!("building mesh");
    let mesh = OrbifoldMesh(InnerType::Mesh(ClosedTriangleMesh::build(builder).unwrap()));

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

    let camera_and_light_transform = Transform::from_xyz(3.8, 3.8, 3.8).looking_at(Vec3::ZERO, Vec3::Y);
    // let (yaw, pitch, _) = camera_and_light_transform.rotation.to_euler(EulerRot::XYZ);
    commands.spawn((Camera3d::default(), camera_and_light_transform, CameraRotation { yaw: 0.0, pitch: 0. }));
    commands.spawn((PointLight::default(), camera_and_light_transform));

    // window.cursor_options.grab_mode = CursorGrabMode::Locked; 
    // window.cursor_options.visible = false;

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