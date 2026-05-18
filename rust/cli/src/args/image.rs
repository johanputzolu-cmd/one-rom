// Copyright (C) 2026 Piers Finlayson <piers@piers.rocks>
//
// MIT License

//! Argument definitions for `onerom image`.

use crate::args::CommandTrait;
use clap::{Args, Subcommand};
use enum_dispatch::enum_dispatch;

#[derive(Debug, Args)]
pub struct ImageArgs {
    #[command(subcommand)]
    pub command: ImageCommands,
}

impl CommandTrait for ImageArgs {
    fn requires_device(&self) -> bool {
        self.command.requires_device()
    }
}

#[enum_dispatch(CommandTrait)]
#[derive(Debug, Subcommand)]
pub enum ImageCommands {
    /// Swap adjacent byte pairs in a ROM image file.
    ///
    /// Reverses the byte order within each 16-bit word throughout the image.
    /// Required for 16-bit wide ROM types (e.g. 27C400) when the source image
    /// has bytes in the opposite order to that expected by One ROM.
    ///
    /// The input file must have an even number of bytes.
    ///
    /// Example:
    ///
    ///   onerom image swap-bytes --input kick.bin --output kick-swapped.bin
    SwapBytes(ImageSwapBytesArgs),
}

#[derive(Debug, Args)]
pub struct ImageSwapBytesArgs {
    /// Input ROM image file.
    #[arg(long, short, visible_alias = "in", value_name = "FILE")]
    pub input: String,

    /// Output file path.
    #[arg(long, short, visible_alias = "out", value_name = "FILE")]
    pub output: String,
}

impl CommandTrait for ImageSwapBytesArgs {
    fn requires_device(&self) -> bool {
        false
    }
}