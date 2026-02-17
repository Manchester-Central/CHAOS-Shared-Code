package com.chaos131.tables;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import java.util.List;
import java.util.function.Function;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class LookupTableTests {

  private class FakeTableRow implements ITableRow<DistanceUnit, Distance> {

    private Distance m_distance;
    private LinearVelocity m_launchSpeed;

    public FakeTableRow(Distance distance, LinearVelocity launchSpeed) {
      m_distance = distance;
      m_launchSpeed = launchSpeed;
    }

    @Override
    public Distance getMeasure() {
      return m_distance;
    }
  }

  private class FakeLookupTable extends LookupTable<DistanceUnit, Distance, FakeTableRow> {
    @Override
    public FakeTableRow mergeRows(Distance targetMeasure, FakeTableRow row1, FakeTableRow row2) {
      var linearVelocity =
          interpolate(
              targetMeasure,
              row1,
              row2,
              (row) -> ((FakeTableRow) row).m_launchSpeed.in(MetersPerSecond));
      return new FakeTableRow(targetMeasure, MetersPerSecond.of(linearVelocity));
    }
  }

  @BeforeEach
  public void clean() {}

  @Test
  public void testLookupTableDirectlyRelated() {
    var table =
        new FakeLookupTable()
            .addRows(
                List.of(
                    new FakeTableRow(Meters.of(0), MetersPerSecond.of(0)),
                    new FakeTableRow(Meters.of(10), MetersPerSecond.of(100))));

    Function<Distance, Double> getLookupMeters =
        (Distance d) -> table.performLookup(d).m_distance.in(Meters);
    Function<Distance, Double> getLookupSpeed =
        (Distance d) -> table.performLookup(d).m_launchSpeed.in(MetersPerSecond);

    // Check main measure (Distance) values
    assertEquals(
        0, // TODO: swap expected/actual
        getLookupMeters.apply(Meters.of(-1))); // If less than lowest value, expect lowest value
    assertEquals(
        10,
        getLookupMeters.apply(Meters.of(11))); // If higher than highest value, expect highest value
    assertEquals(
        0, getLookupMeters.apply(Meters.of(0))); // If exact value, expect exact value (low)
    assertEquals(
        10, getLookupMeters.apply(Meters.of(10))); // If exact value, expect exact value (high)
    assertEquals(5, getLookupMeters.apply(Meters.of(5))); // If inbetween value, expect same value

    // Check interpolated measure (LaunchSpeed) values
    assertEquals(
        0, getLookupSpeed.apply(Meters.of(-1))); // If less than lowest value, expect lowest value
    assertEquals(
        100,
        getLookupSpeed.apply(Meters.of(11))); // If higher than highest value, expect highest value
    assertEquals(0, getLookupSpeed.apply(Meters.of(0))); // If exact value, expect exact value (low)

    assertEquals(
        100, getLookupSpeed.apply(Meters.of(10))); // If exact value, expect exact value (high)
    assertEquals(50, getLookupSpeed.apply(Meters.of(5))); // If inbetween value, expect same value
  }
}
