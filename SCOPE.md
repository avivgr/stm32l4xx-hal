# Scope: bumping `stm32l4` PAC from 0.14.0 to 0.16.0

Branch `bump_0.16`, based on clean `origin/master` (`bf2ded1`). No local feature
patches are included.

## Why this came up

`origin/master` **does not compile on rustc 1.88**: six `invalid_reference_casting`
hard errors from casting `&T` to `*mut T` when writing byte-wide values to the
USART, SPI and QSPI data registers.

Open upstream PR
[#351](https://github.com/stm32-rs/stm32l4xx-hal/pull/351) proposes fixing this
with `UnsafeCell::raw_get`. On that PR, burrbull (stm32-rs) commented on
2025-12-25 that `stm32l4` v0.16 exposes dedicated `dr8`/`dr16` registers and
that this is the fix he would prefer. Confirmed in the 0.16 sources:

- `spi1::RegisterBlock::dr8()`
- `quadspi::RegisterBlock::dr8()`, `dr16()`
- USART keeps `tdr()`/`rdr()`; with the 0.16 API a plain `.write()` is enough,
  so no cast is needed there either.

Taking burrbull's fix therefore requires the PAC bump. This document scopes it.

## Version landscape

| Version | Published | Note |
| --- | --- | --- |
| 0.16.0 | 2025-05-26 | latest; has `dr8`/`dr16` |
| 0.15.1 | 2022-07-04 | what open PR #352 bumps to |
| 0.15.0 | 2022-07-04 | yanked |
| **0.14.0** | 2021-10-03 | **currently pinned** (`Cargo.toml:26`) |

PR #352 is not a useful reference for this work. Its 745-line diff across 16
files is almost entirely uncommenting `// feature = "stm32l4r5"` gates to add
L4R5/L4S5 support, plus a one-line PAC bump. **0.14 to 0.15 required no source
changes at all.** The entire API break is 0.15 to 0.16, which nobody upstream
has attempted. PR #352 will conflict with this work on `Cargo.toml` and the
feature list, but offers no migration guidance.

## Method

1. Bumped the dependency to `0.16.0`.
2. Built `--lib --target thumbv7em-none-eabihf --features rt,unproven,stm32l432`:
   **855 errors**.
3. Applied an automated rewrite driven by rustc's own diagnostic spans
   (`--message-format=json`): for every `E0616` (private field) and `E0615`
   (method, not a field), append `()` at the reported span end. Iterated to a
   fixed point.
4. Hand-fixed the three macro bodies that pattern could not reach, because the
   register name arrives as a `$ident` macro argument:
   - `bus_struct!` in `src/rcc.rs`
   - `generate_register!` in `src/flash.rs`
   - `af!` in `src/gpio.rs`

**Result: 855 errors to 192, with 491 call sites rewritten automatically.**

So roughly 77% of the breakage is mechanical churn from svd2rust making
register-block fields private behind accessor methods (`.ccr` becomes `.ccr()`).

## What remains: 192 errors, 8 classes

| Class | Count | Nature |
| --- | ---: | --- |
| DMA channel cluster migration | 126 | Real design change |
| Writer closure return type | 26 | Mechanical |
| `bits()` now unsafe (E0133) | 8 | Mechanical |
| Write-1-to-clear fields | 8 | Small semantic change |
| Renamed `ADC_CCR` fields | 6 | Needs reference-manual lookup |
| Macro-arg damage from the automated pass | 5 | Revert arg, fix macro body |
| `_SPEC` type renames | 4 | Mechanical |
| `Periph<>` trait bounds (E0277) | 2 | Needs investigation |

### The three that are not typing exercises

**DMA is the bulk of the work.** 0.16 clusters the channel registers. What was
`ccr1, cndtr1, cpar1, cmar1, ccr2, ...` is now `ch(n).cr()`, `ch(n).ndtr()`,
`ch(n).par()`, `ch(n).mar()`, with `ch1()`..`ch7()` convenience accessors. The
HAL's `dma!` macro currently takes 13 per-channel register identifiers times 7
channels as literal arguments (`src/dma.rs:1151` onward). That invocation and
the macro consuming it are replaced by index-based access. The result should be
materially shorter than what is there today, but it is a rewrite of the crate's
most intricate module.

**Peripherals are now type aliases, not structs:**

```rust
pub type ADC1 = crate::Periph<adc1::RegisterBlock, 0x5004_0000>;
```

Distinct addresses keep them distinct types, so most `impl Trait for pac::SPI1`
survives. But the two `E0277`s at `src/adc.rs:519` show at least one
instance-generic impl no longer lines up. Depth is unknown until the DMA
cascade clears.

**Write-1-to-clear fields are now typed `Bit1C`,** so `.set_bit()` is gone in
favour of `.clear_bit_by_one()`. This is the PAC enforcing a correctness
distinction the old API let you get wrong. Affects flag clearing in `adc.rs`
and `i2c.rs`.

## Read on effort

The automated pass did the easy 77%. Two things to keep in mind about the
remaining 192:

- It is for **one device feature** (`stm32l432`) and **lib only**.
- It covers neither the **18-device CI matrix** nor the **~40 examples**.

Unmeasured tail risk: device-specific `cfg` blocks (the L4+ parts, `l4r9`, have
a different DMA with DMAMUX), and every example that touches a changed API.

Realistic estimate:

- DMA rewrite: a solid focused session.
- The mechanical classes: about an hour.
- Per-device and per-example fixing: open-ended.

## Benefits beyond community service

Verified against the 0.16 sources, these matter for downstream projects
independently of whether upstream ever merges anything:

1. **Typed, per-device trigger enums.** `adc1::cfgr::EXTSEL` is now a generated
   enum (`Tim1Cc1`, `Tim6Trgo`, `Tim15Trgo`, ...) derived from each device's own
   SVD. In 0.14 it is a bare `u8` field. This directly replaces a hand-written
   `ExternalTrigger` enum carrying raw encodings, removes the `unsafe { bits() }`
   around it, and structurally eliminates the risk of applying one device's
   trigger map to another ADC instance.
2. **Write-1-to-clear is type-enforced** via `Bit1C`, catching a whole class of
   status-flag bugs at compile time.
3. **No pointer casts needed** for byte-wide data-register access: `dr8()`/`dr16()`
   replace `&reg as *const _ as *mut u8` outright.
4. **`critical-section` is a default feature**, which is the modern interop story
   for RTIC 2 and embassy.
5. **`defmt` and `atomics` features** are available for register types.
6. **Four years of stm32-rs SVD corrections**, meaning more typed fields and
   fewer raw-bits escapes in downstream code generally.

## Status of this branch

The WIP commit is the mechanical pass only. **It does not compile.** It is a
scoping artifact, not a proposed change.
