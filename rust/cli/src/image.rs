// Copyright (C) 2026 Piers Finlayson <piers@piers.rocks>
//
// MIT License

//! Implementation of `onerom image` subcommands.

use crate::args::image::ImageSwapBytesArgs;
use onerom_cli::{Error, Options};

pub async fn cmd_swap_bytes(options: &Options, args: &ImageSwapBytesArgs) -> Result<(), Error> {
    if options.verbose {
        println!("Reading ROM image from {} ...", args.input);
    }
    let data = std::fs::read(&args.input).map_err(|e| Error::io(&args.input, e))?;

    if data.len() % 2 != 0 {
        return Err(Error::OddLengthImage(args.input.clone(), data.len()));
    }

    let swapped: Vec<u8> = data.chunks_exact(2).flat_map(|w| [w[1], w[0]]).collect();

    std::fs::write(&args.output, &swapped).map_err(|e| Error::io(&args.output, e))?;

    if options.verbose {
        println!(
            "Wrote {} bytes to {} with byte pairs swapped",
            swapped.len(),
            args.output
        );
    } else {
        println!("Written to {}", args.output);
    }

    Ok(())
}
