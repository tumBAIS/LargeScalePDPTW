use std::fmt::Display;
use std::fs::File;
use std::io::{BufWriter, Write};

use kdsp::Arc;

pub fn write_instance<W>(
    path: impl Into<String>,
    name: impl Into<String>,
    num_nodes: impl Into<usize>,
    arcs: Vec<Arc<W>>,
    kdsp: Option<Vec<Vec<usize>>>,
) -> anyhow::Result<()>
where
    W: Display,
{
    let f = File::create(path.into())?;
    let mut file = BufWriter::new(&f);

    writeln!(file, "{}", name.into())?;
    writeln!(file, "{}", num_nodes.into())?;
    writeln!(file, "{}", arcs.len())?;

    writeln!(file, "")?;

    for it in arcs {
        writeln!(
            file,
            "{from},{to},{w}",
            from = it.from,
            to = it.to,
            w = it.w,
        )?;
    }

    if let Some(paths) = kdsp {
        writeln!(file, "")?;
        for path in paths {
            writeln!(
                file,
                "{}",
                path.iter()
                    .map(|it| it.to_string())
                    .collect::<Vec<String>>()
                    .join(",")
            )?;
        }
    }

    Ok(())
}
