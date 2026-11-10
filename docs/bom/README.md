# Bill of materials

`bom.csv` is the source of truth for the under-\$150k claim and for the build guide. CSV
because it diffs in git, opens in any spreadsheet, and the paper's table generates from it
rather than being maintained twice.

## Three rules that are not stylistic

**List price, or the standard quoted price where a product is sold only by quote. Never a
special discount.** Not what we paid. Distributor terms routinely make pricing confidential,
and a negotiated discount is not reproducible by a reader. Where a public price exists it is
used. Where the product is sold only by quote (the UR arms, the Haply kits), the row carries
the vendor's standard per-unit price before any discount (`price_basis` `quote`), and the
notes say the price is subject to each institution's quote. Extra discounts won by asking
further are never used.

**No vendor part lists reproduced wholesale.** Where a vendor sells a configured assembly,
the BOM carries *one line item plus a link*, not their itemized parts. The pedestal is ~300
Vention part numbers; reproducing that list would republish the vendor's BOM, and a reader
cannot hand-assemble from it anyway. One line and a public design link is both more useful
and cleaner.

**Quotes stay out of this repo.** They carry names, email addresses, shipping addresses and
institution names, any of which is a desk rejection under double-blind review. Keep them in
`$ARTS_LOCAL/`.

## The one exception to list prices: Intuitive parts are costed used

The da Vinci instruments, the Si endoscope, its light guide and drape have no list price a
researcher can pay: Intuitive does not sell them to labs. ARTS's came from the research
surplus market (eBay), expired for clinical use, and are used for research only. Those rows
are `used`, priced at a figure a reproducer can actually find on current listings (the da
Vinci X/Xi instruments at 455.00 each, set by the PI from listings on 2026-11-02), with the
listing in `url`. The same applies to other clinical surplus bought used (light source,
imaging tower cart). The paper states this costing rule next to the total.

## Procurement constraints — the UR arms

Universal Robots' own documentation lists medical use as an *unintended use* and names
surgery, treatment, diagnostics, healthcare and "any use in proximity to patient/s"
explicitly ([UR user manual, Liability and Intended Use][ur-intended-use]). A group
reproducing ARTS should therefore disclose the research application when requesting a quote
and buy through the channel UR designates for it; ARTS's arms came through Kinova. The arms
are standard UR5e/UR5, and the control stack (`ur_robot_driver`, `ur_description`,
`ur_ikfast`) is unaffected.

**The UR5e is costed at 30,000.00 per arm, its typical standard quoted price** (2026-11-02):
quotes from four independent vendors all came out at about that figure before any discount,
and none publishes an online price, so it is a sold-by-quote product like the Haply kits.
The 34,527.00 that Vention still shows for a UR5e marked Obsolete is kept in the row notes as
the upper bound. Pricing is Ex-Works, so **freight is on top of every arm line** and the BOM
total is a floor. Quotes expire, so confirm the figure with your own quote.

[ur-intended-use]: https://www.universal-robots.com/manuals/EN/HTML/SW5_24/Content/prod-usr-man/complianceUR20/H_g5_sections/safety_g5/intended_use.htm

## Price formation (2026-11-02)

Where the total comes from, so a reader can see which lines decide the under-\$150k claim.
The rule for every row is the price a reader can actually pay today for what ARTS uses (the
exact configuration, not a bigger one), from the cheapest legitimate source found; reused or
standard lab equipment is listed but not costed (`lab` rows).

By station:

| Station | USD |
|---|---|
| Patient cart (pedestal, 3 arms, 2 instrument adapters, instruments, endoscope and stereo coupling, main PC, ARM HOLD relay, switch) | 119,523.13 |
| Support tower (tower cart and monitor, ESU + ESU-TAP, light source, smoke evacuator + filter, irrigation, suction) | 2,746.17 |
| Surgeon console (80/20 frame, 2 Haply kits, HMD, armrest, footswitches) | 14,731.27 |
| Supervisor station (2 monitors, ARM HOLD button box) | 319.00 |
| **Priced total** | **137,319.57** |

By category:

| Category | USD | Share | What drives it |
|---|---|---|---|
| Arms | 90,000.00 | 65.5 % | 3x UR5e at the typical standard quote, 30,000.00 each |
| Structure | 18,355.82 | 13.4 % | Vention pedestal parts 16,541.55; 80/20 console frame 1,504.27; used tower cart; armrest |
| Haptics | 11,000.00 | 8.0 % | 2x Haply Inverse3 Developer Kit (with VerseGrip) at the standard 5,500.00 |
| Adapters | 3,234.38 | 2.4 % | 2x first-party instrument adapter, ~1,617.19 each (bought parts, CNC shafts, filament) |
| Compute | 3,679.99 | 2.7 % | main PC, the exact 1 TB configuration at dell.com |
| Cameras | 2,242.00 | 1.6 % | 2x BFS-U3-63S4C-C + 2x 50 mm lens |
| Energy | 2,021.73 | 1.5 % | used ESU, used ESU pedals, ESU-TAP + supply |
| Instruments | 1,820.00 | 1.3 % | 4 da Vinci X/Xi instruments, used, 455.00 each |
| Display, optics, support, safety, input, network | 4,965.65 | 3.6 % | HMD and monitors, stereo coupling and used endoscope, used smoke evacuator + filter and pumps, ARM HOLD, Savant pedals, switch |

- **The arms are the system's cost** (66 %). They are sold only by quote, so they are costed at
  the typical standard quote, 30,000.00 each, subject to each institution's quote; the
  34,527.00 upper bound is in the row notes. All three are costed as UR5e although ARTS
  measured the camera arm on an existing UR5 CB3, which cannot be bought new.
- **Everything ARTS designed is cheap.** The two instrument adapters, the stereo coupling, the
  ESU-TAP and the camera sync box together are under 4 % of the total.
- **Clinical parts are costed used** (rule above), the smoke evacuator included; its filter
  is a new consumable.
- **Not costed** (`lab`): trainer box, cannulas, the supervisor table, and the measurement
  equipment.
- **Still TBD:** the VerseGrip grip add-on and the camera head drape, with 12,680.43 USD left
  under 150,000.

## Columns

| Column | Meaning |
|---|---|
| `cart` | the station: `patient` (patient cart), `tower` (support tower), `console` (surgeon console), `supervisor` (supervisor station), or `lab` |
| `category` | rough grouping for the paper table |
| `item` | plain description a reader can shop from |
| `vendor` | `ARTS (first-party)` for things we designed |
| `part_number` | orderable identifier; `TBD` until confirmed |
| `qty` | count for one complete system |
| `unit_price_usd` | per `price_basis`; `TBD` if not yet confirmed |
| `price_basis` | `list`, `quote` (standard price of a sold-by-quote product), `estimate`, `used`, `TBD` — never a special discount |
| `url` | product or public design page |
| `notes` | substitutions, caveats, why a line is excluded from the total |

`lab` rows are excluded from the system cost: measurement equipment used to *evaluate* ARTS,
and standard equipment a surgical-robotics lab already has (trainer box, cannulas).

## Filling in the TBDs

Most rows are `TBD` on purpose rather than filled with plausible-looking numbers. Fill each
from a vendor's public price page, and record the URL in the same row so the figure can be
re-checked later. If a price is only available on request, mark `price_basis` as `estimate`
and say so in the notes — an honest estimate is fine, an unsourced precise number is not.
