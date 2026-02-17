package com.chaos131.tables;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import java.util.List;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class LookupTableTests {

  private class FakeTableData implements ITableData<DistanceUnit, Distance> {

    private Distance m_distance;
    private LinearVelocity m_launchSpeed;

    public FakeTableData(Distance distance, LinearVelocity launchSpeed) {
      m_distance = distance;
      m_launchSpeed = launchSpeed;
    }

    @Override
    public Distance getMeasure() {
      return m_distance;
    }

    @Override
    public FakeTableData mergeData(
        Distance targetMeasure,
        ITableData<DistanceUnit, Distance> data1,
        ITableData<DistanceUnit, Distance> data2) {
      var linearVelocity =
          LookupTable.interpolate(
              targetMeasure,
              data1,
              data2,
              (td) -> ((FakeTableData) td).m_launchSpeed.in(MetersPerSecond));
      return new FakeTableData(targetMeasure, MetersPerSecond.of(linearVelocity));
    }
  }

  @BeforeEach
  public void clean() {}

  @Test
  public void testMirroringX() {
    var table = new LookupTable<DistanceUnit, Distance, FakeTableData>();
    table.addData(
        List.of(
            new FakeTableData(Meters.of(0), MetersPerSecond.of(0)),
            new FakeTableData(Meters.of(10), MetersPerSecond.of(100))));

    // Check main measure (Distance) values
    assertEquals(
        table.performLookup(Meters.of(-1)).m_distance.in(Meters), // TODO: swap expected/actual
        0); // If less than lowest value, expect lowest value
    assertEquals(
        table.performLookup(Meters.of(11)).m_distance.in(Meters),
        10); // If higher than highest value, expect highest value
    assertEquals(
        table.performLookup(Meters.of(0)).m_distance.in(Meters),
        0); // If exact value, expect exact value (low)
    assertEquals(
        table.performLookup(Meters.of(10)).m_distance.in(Meters),
        10); // If exact value, expect exact value (high)
    assertEquals(
        table.performLookup(Meters.of(5)).m_distance.in(Meters),
        5); // If inbetween value, expect same value

    // Check interpolated measure (LaunchSpeed) values
    assertEquals(
        table.performLookup(Meters.of(-1)).m_launchSpeed.in(MetersPerSecond),
        0); // If less than lowest value, expect lowest value
    assertEquals(
        table.performLookup(Meters.of(11)).m_launchSpeed.in(MetersPerSecond),
        100); // If higher than highest value, expect highest value
    assertEquals(
        table.performLookup(Meters.of(0)).m_launchSpeed.in(MetersPerSecond),
        0); // If exact value, expect exact value (low)
    assertEquals(
        table.performLookup(Meters.of(10)).m_launchSpeed.in(MetersPerSecond),
        100); // If exact value, expect exact value (high)
    assertEquals(
        table.performLookup(Meters.of(5)).m_launchSpeed.in(MetersPerSecond),
        50); // If inbetween value, expect same value
  }
}
