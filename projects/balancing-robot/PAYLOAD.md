# Step 4: rigid payload variation

Implemented 12 November 2026. Author: Michael Agbo. Simulation-only research.

## Open and use

Open [the live 3D lab](visualisation/index.html). The robot view sits beside the
compact controls; on smaller screens it stays visible as you scroll the controls.
Sensor settings expand when needed. Under **Carry a payload**, set
added mass (0-1.5 kg) and payload centre height (0.10-0.40 m along the body from
the axle). Changing either resets the run. Choose **Perfect state** to isolate
loading, or retain the Step 3 noise/filter controls for combined experiments.
Run the simulation, then push left/right.

The gold box is the payload, and the yellow point is the combined body COM. The
force arrow starts there. The purple outline shows the estimated body and load
pose. Export run JSON records the nominal configuration, loaded parameters,
payload dimensions, sensor settings, true/estimated trajectories and force events.
Recorded experiments retain their original unloaded model. Camera directions do
not change the ground-X force direction.

## Physical model

The payload is a rigid cuboid: 0.08 m fore-aft width, 0.06 m height along the body,
and 0.12 m lateral depth. Its centre lies on the body axis at distance h from the
axle. It is fixed before the run; this is not online pickup or moving cargo.
Dimensions are fixed and density varies with mass. The attachment is massless.
The original shell is illustrative, and a ballast box may overlap that envelope.

For original body mass m0, COM distance l0 and COM pitch inertia I0:

```text
M = m0 + mp
L = (m0*l0 + mp*h) / M
Ip = mp*(0.08^2 + 0.06^2) / 12
I = I0 + m0*(l0-L)^2 + Ip + mp*(h-L)^2
```

These update every mass, COM and inertia term in the existing nonlinear plant.
Zero payload restores original parameters exactly. Base mass and wheel inertia
are additional to the combined body mass displayed in the lab. Gains, integral
limits, motor torque limits and Kalman process covariance are not redesigned.

**Force convention:** each horizontal push acts at the combined loaded-body COM.
The moment is F*L*cos(tilt), so the application point changes with loading. This
is consistent with the original COM-force model, but it is not a fixed chassis
attachment-point test. Do not attribute all response changes to mass alone.

## Frozen experiment

[Protocol](payload_protocol.json): exploratory, fixed before execution, not
preregistered or held out. The controller gains remain nominal.

- 21 distinct loads: zero once, plus five positive masses at four heights.
- 2,016 deterministic trials: 21 loads x 4 tilts x 4 forces x 3 torque limits x
  2 controllers. Each load/controller has 48 common conditions.
- 2,160 sensing trials: 4 loads x 3 scenarios x 3 observation profiles x
  2 controllers x 30 paired seeds. Noise SDs are 0.5 deg and 2 deg/s; profiles
  are direct at 0 ms, Kalman at 0 ms and Kalman at 10 ms.
- Separate audit: all 2,016 deterministic trials repeated with two RK4 substeps,
  retaining 200 Hz control and sensing. No outcome labels changed.

Primary outcome: fall by 10 seconds at a control endpoint. Original recovery
requires a final continuous one-second dwell inside tilt, position and speed
bounds. Final-second RMS metrics are additional descriptions, not new recovery
labels. They use 201 endpoint samples from 9 to 10 seconds and are null for falls.
Wilson intervals apply only to noisy fixed-condition fall fractions; IQR bars
show seed variation. The deterministic grid is not a probability distribution
for deployment. Squared torque is not electrical energy.

## Deliverables

- [HTML report](results/payload/report.html)
- [PDF supplement](publication/output/pdf/payload_report.pdf)
- [Trial records](results/payload/results.csv)
- [Ideal grid summaries](results/payload/ideal_summary.csv)
- [Sensing summaries](results/payload/sensing_summary.csv)
- [Integration audit](results/payload/integration_audit.json)
- Seven `publication/figures/step4_*` figures in PDF, SVG and 600 dpi PNG
- Editable LaTeX in `publication/output/latex/payload_report.tex` (not compiled)

## Reproduce and verify

```powershell
node run_payload.js
node run_payload_audit.js
publication/.venv/Scripts/python.exe publication/build_payload_report.py
node tests/test_payload.js
node tests/test_3d_ui.js
publication/.venv/Scripts/python.exe -m unittest discover -s tests -v
```

The snapshot records source hashes before the run. The shared module
`visualisation/payload.js` feeds both the interactive and batch simulations.
Independent checks cover the component mass matrix, inertia identities and six
selected Python trajectories. UI tests use real Three.js geometry with a renderer
mock. Browser access was unavailable, so interactive WebGL pixels were not
visually checked. PDF and static figure QA are recorded separately.

## Limits and next comparison

The model remains planar and rigid with no slip, collisions, motor electrical
response, off-axis payload, hardware calibration or online load estimation.
Only nonnegative initial tilts and force directions are tested in this sweep.
A [payload-aware LQR comparison](COMPARISON.md) now evaluates these frozen gains
against a fixed-cost redesign on reserved conditions, with a zero tuning-search
budget and explicit estimate errors.
Independent simulator or hardware validation is still needed for physical claims.
Original manuscript and Step 3 datasets are kept separate from this supplement.

Methods references: [MIT, parallel-axis theorem](https://ocw.mit.edu/courses/8-01sc-classical-mechanics-fall-2016/resources/29-3/)
and [Baek et al., unknown payload estimation (2024)](https://arxiv.org/abs/2403.10948v2).
