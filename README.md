Struktur des Codes:

Mesh-Konstruktion und HEC-> src/mesh
Berechnung der projektiven Struktur -> src/projective_structure/mod.rs (vorallem struct CEquations)
Visualisierung und rückgängig machen von HEC in PS -> src/projective_structure/visualisation.rs
Einstieg des Programms -> src/main.rs

Ausführen mit `cargo run --release`.

Tastenkombinationen:

x,y,z -> drehen
w,a,s,d, leertaste, Umschalt-Links -> Bewegen vor, links, rechts, zurück, hoch, runter
p -> Projektive struktur berechnen
c -> HEC
u -> rückgängig machen
, -> vorheriger iterationsschritt projektive struktur
. -> nächster
0 -> erster