# Flight Log Archive

Rocket and Ground flight records are archived outside the firmware repository
at:

```text
/Users/k_pochkaev/github/flight_logs
```

From this repository, the same directory is `../../flight_logs`. Keeping the
archive separate prevents large binary logs, generated CSV files, and HTML
reports from entering the firmware Git history.

## Directory layout

Use one directory per device and launch/campaign date:

```text
flight_logs/
├── rocket_v10/
│   └── 16Aug/
└── ground_v10/
    └── 16Aug/
```

The date directory is the local launch/campaign date label written as
`<day><three-letter month>` with no leading zero, for example `9May`, `21Jun`,
`26Jul`, or `16Aug`. Keep related setup and post-flight sessions with that
campaign even when a firmware UTC filename falls on the adjacent calendar day.

Create matching Rocket and Ground date directories when both devices were used
for a launch. Do not combine their files: Rocket NAND is the authoritative
onboard record, while Ground SD is independent telemetry, link, and launch
controller corroboration.

## Files to preserve

For Rocket V10, keep:

- the original `rocket_fltNNNN.bin` NAND download
- the main `rocket_nand_NNNN_opID.csv` export
- every available detail export using the same base name:
  - `_imu.csv`
  - `_mag.csv`
  - `_baro.csv`
  - `_gps.csv`
  - `_batt.csv`
  - `_event.csv`
  - `_telem.csv`
  - `_att.csv`
- the generated analyzer report named
  `rocket_nand_NNNN_opID_analysis.html`

For Ground V10, copy every original `ground_*.log` file from the flight date.
Preserve the firmware-generated filenames and file contents. A date directory
may contain setup, aborted, and post-flight sessions; retain them until the
flight has been correlated by timestamp and telemetry evidence.

## Rocket collection procedure

Keep the Rocket powered and physically SAFE. Finalize the flight before reading
it: normally `LANDED` does this automatically. If the flight remains in a
nonterminal state, return to SAFE and hold the local service button for about
two seconds; the firmware commits RAM to NAND before rebooting. Do not remove
power to force a reset because uncommitted flight RAM would be lost.

When using the external 3.3 V USB-UART adapter on the GPS connector:

1. Cross TX/RX and connect a common ground; do not connect adapter VCC when the
   Rocket has its own power.
2. At `9600 8N1`, send `SERVICE UART CONFIRM`.
3. Wait for `SERVICE SWITCH 115200`, then reopen the adapter at `115200 8N1`.
4. Run read-only inventory commands first:

   ```text
   STORAGE STATUS
   NAND LIST
   NAND INFO <index>
   ```

Use the checksummed, resumable client to preserve the raw NAND file in a
temporary staging directory:

```bash
repo=/Users/k_pochkaev/github/flight_computers/teensy_v0
archive=/Users/k_pochkaev/github/flight_logs
port=/dev/cu.usbserial-...
index=20
flight_day=16Aug
staging=/Users/k_pochkaev/Downloads/RocketV10-flight-staging
raw_name=$(printf 'rocket_flt%04d.bin' "$index")

mkdir -p "$staging" "$archive/rocket_v10/$flight_day"
python3 "$repo/rocket/rocket_v10/tools/flight_storage.py" \
  --port "$port" download nand "$index" \
  "$staging/$raw_name" --resume
```

The client verifies CRC32 for each transfer frame and supports resuming at the
existing local byte offset. Confirm that the final local size matches
`NAND INFO`.

Generate the full CSV set on the Rocket without erasing NAND:

```text
NAND EXPORT SD <index> FULL
SD LIST
```

Download the generated main and detail CSV files with the same client using
`download sd <filename> <output> --resume`. Copy the completed files rather
than moving them so the staging copy remains available until archive
verification is complete.

For example, after downloading flight 20 and its operation `406562` CSV set to
the staging directory:

```bash
destination="$archive/rocket_v10/$flight_day"
export_base=rocket_nand_0020_op406562

cp -p "$staging/$raw_name" "$destination/"
cp -p "$staging/$export_base"*.csv "$destination/"
cmp "$staging/$raw_name" "$destination/$raw_name"
```

Replace `export_base` with the exact main CSV base reported by `SD LIST`.
Generate the interactive report from the archived main CSV:

```bash
main="$destination/$export_base.csv"
python3 "$repo/visualizer/rocket_v10_log_analyzer.py" "$main" \
  -o "${main%.csv}_analysis.html"
```

## Ground collection procedure

Ground logs may be copied directly from its SD card or downloaded through its
SAFE serial storage interface. Place all files for the local flight date in:

```text
/Users/k_pochkaev/github/flight_logs/ground_v10/<dayMonth>/
```

Ground serial reads require both ARM switches SAFE and both START buttons
released. The relevant commands are:

```text
STORAGE STATUS
SD LIST
SD INFO <filename>
SD READ <filename> <offset> <length>
```

Use `flight_storage.py download sd ... --resume` for checked serial transfers.
Do not rename the original Ground logs; their UTC timestamp and log index are
needed for Rocket/Ground correlation.

## Verification and retention

Before erasing any device storage:

1. Confirm the raw Rocket binary size matches `NAND INFO`.
2. Confirm every serial download completed its CRC check.
3. Compare staged and archived files with `cmp` or SHA-256.
4. Open the analyzer report and confirm it references all expected detail CSV
   streams.
5. Preserve both Rocket and Ground originals even when one side has missing or
   stale telemetry.

Never erase Rocket NAND or Ground SD merely because an export command reported
success. Erase only after the independent archive copy has been verified.
