## IMU (AHRS raw data)

**Acceleration.csv**

* ax, ay, az [m/s²]
* Raw specific force (includes gravity).
  → Use for INS mechanization.

**AngularVelocity.csv**

* wx, wy, wz [rad/s]
  → Gyro rates in body frame.

**MagneticField.csv**

* mx, my, mz [Gauss]
  → Magnetometer for heading correction.

**EulerAngles.csv**

* roll, pitch, yaw [rad]
  → Fused AHRS orientation (already filtered).

**EulerAnglesDelta.csv** *(if present)*
→ Incremental rotation.

---

## DVL

**GroundVelocity.csv**

* vx, vy, vz [m/s] (bottom-track)
  → Main DVL velocity in body frame.
  → Used for velocity aiding in INS.

**WaterVelocity.csv**

* Velocity relative to water.
  → Used for current estimation.

**Distance.csv**

* Beam range measurements [m].
  → Used for altitude estimation.
  → Each beam separately.

**DvlRejection.csv**
→ Logs when DVL measurements were rejected.
→ Important for debugging estimator instability.
→ Helps detect bottom loss / bad correlation.

---

## Sonar

**SonarData.csv**
→ Raw sonar packets (Sidescan / Echo / Multibeam).
→ Contains intensity or range data depending on type.
→ Large file (as expected).

From message type field:

* type = SIDESCAN → seabed image
* type = ECHOSOUNDER → depth
* type = MULTIBEAM → bathymetry swath

---

## Related but useful

**Depth.csv**
→ Pressure-based depth sensor.

**SoundSpeed.csv**
→ Needed for correct sonar/DVL scaling.

**NavigationUncertainty.csv**
→ Estimator covariance / uncertainty.

**EstimatedState.csv**
→ Full fused navigation solution (position, velocity, attitude).

---

If you’re building a full navigation stack (INS + DVL + sonar SLAM):

**State estimation core (local XYZ pose):**
Use → `Acceleration.csv` + `AngularVelocity.csv` (INS propagation)

* `GroundVelocity.csv` (DVL bottom-track correction)
* `Depth.csv` (absolute Z from pressure)

**Quality gating:**
Use → `DvlRejection.csv` to detect bottom loss / invalid velocity and avoid corrupting the filter.

**Mapping / SLAM layer:**
Use → `SonarData.csv` + `Distance.csv` (beam ranges)

* DVL velocity + Depth for scan alignment and map building.

INS propagates → DVL corrects velocity → Depth stabilizes Z → Sonar builds map → SLAM feedback can refine pose.

NOTE: All this info is taken from this website and extracted using neptune toolchain:
https://www.lsts.pt/docs/imc/imc-5.4.30/
https://github.com/LSTS/neptus
https://github.com/LSTS

