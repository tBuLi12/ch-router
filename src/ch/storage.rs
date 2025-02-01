use std::{fs::File, io, path::Path};

use super::{unfolding::AllEdges, ContractionHierarchy, Edge, ShortcutVia};

impl ContractionHierarchy {
    pub fn save(&self, path: impl AsRef<Path>) -> Result<(), io::Error> {
        self.write(&mut io::BufWriter::new(File::create(path)?))
    }

    pub fn load(path: impl AsRef<Path>) -> Result<Self, io::Error> {
        Self::read(&mut io::BufReader::new(File::open(path)?))
    }

    pub fn write(&self, writer: &mut impl io::Write) -> io::Result<()> {
        writer.write_all(&(self.forward_edges.len() as u32).to_le_bytes())?;

        let mut write_edges = |edges: &[Vec<Edge>]| -> io::Result<()> {
            for edges in edges {
                writer.write_all(&(edges.len() as u32).to_le_bytes())?;
                for edge in edges {
                    writer.write_all(&edge.to.to_le_bytes())?;
                    writer.write_all(&edge.weight.to_le_bytes())?;
                    edge.shortcut_via.write(writer)?;
                }
            }
            Ok(())
        };

        write_edges(&self.forward_edges)?;
        write_edges(&self.backward_edges)?;

        self.all_edges.write(writer)?;

        Ok(())
    }

    pub fn read(reader: &mut impl io::Read) -> io::Result<Self> {
        let mut bytes = [0u8; 4];

        reader.read_exact(&mut bytes)?;
        let nodes_len = u32::from_le_bytes(bytes);

        let mut read_edges = || -> io::Result<_> {
            let mut edges_for_node = Vec::with_capacity(nodes_len as usize);

            for _ in 0..nodes_len {
                reader.read_exact(&mut bytes)?;
                let edges_len = u32::from_le_bytes(bytes);

                let mut edges = Vec::with_capacity(edges_len as usize);

                for _ in 0..edges_len {
                    reader.read_exact(&mut bytes)?;
                    let to = u32::from_le_bytes(bytes);

                    reader.read_exact(&mut bytes)?;
                    let weight = f32::from_le_bytes(bytes);

                    let shortcut_via = ShortcutVia::read(&mut bytes, reader)?;

                    edges.push(Edge {
                        to,
                        weight,
                        shortcut_via,
                    });
                }

                edges_for_node.push(edges);
            }

            Ok(edges_for_node)
        };

        let forward_edges = read_edges()?;
        let backward_edges = read_edges()?;

        let all_edges = AllEdges::read(&mut bytes, reader)?;

        Ok(Self {
            forward_edges,
            backward_edges,
            all_edges,
        })
    }
}
