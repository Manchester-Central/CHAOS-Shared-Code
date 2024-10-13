package com.chaos131.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import com.chaos131.util.FieldData;
import com.chaos131.util.Quad;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class CameraTransformsTests {
    double DELTA = 0.001;

    // @Test
    // public void testCalculateVisibleCoordinates() {
    //     var source_tag = new Translation3d(-0.0381, 5.547868, 1.368552);
    //     var tag_coords = new Quad[] {new Quad(source_tag)};
        
    //     var camerapose = new Pose3d(
    //         new Translation3d(4.0, 5.547868, 1.368552),
    //         new Rotation3d(0,0,Math.PI)
    //     );
    //     var res = CameraTransforms.CalculateVisibleCoordinates(camerapose, tag_coords, 80, 56);
    //     // special case since the target point should never leave the dead center of the image frame
    //     assertEquals(4, res.size());
    //     assertEquals(0, res.get(0).get(0, 0), DELTA);
    //     assertEquals(0, res.get(0).get(1, 0), DELTA);
    //     // System.out.println(camerapose);
    //     // System.out.println(res.size() + " tags found");

    //     camerapose = new Pose3d(
    //         new Translation3d(5, 5.5, 1.3),
    //         new Rotation3d(0,0,Math.PI)
    //     );
    //     res = CameraTransforms.CalculateVisibleCoordinates(camerapose, tag_coords, 80, 56);
    //     assertEquals(4, res.size());
    //     // System.out.println(camerapose);
    //     // System.out.println(res.size() + " tags found");

    //     camerapose = new Pose3d(
    //         new Translation3d(0, 5.5, 1.3),
    //         new Rotation3d(0,0,Math.PI)
    //     );
    //     res = CameraTransforms.CalculateVisibleCoordinates(camerapose, tag_coords, 80, 56);
    //     assertEquals(0, res.size());
    //     // System.out.println(camerapose);
    //     // System.out.println(res.size() + " tags found");

    //     camerapose = new Pose3d(
    //         new Translation3d(8, 5.5, 1.3),
    //         new Rotation3d(0,0,Math.PI/2)
    //     );
    //     res = CameraTransforms.CalculateVisibleCoordinates(camerapose, tag_coords, 80, 56);
    //     assertEquals(0, res.size());
    //     // System.out.println(camerapose);
    //     // System.out.println(res.size() + " tags found");

    //     camerapose = new Pose3d(
    //         new Translation3d(5, 5, 0),
    //         new Rotation3d(0,Math.PI/20,Math.PI)
    //     );
    //     res = CameraTransforms.CalculateVisibleCoordinates(camerapose, tag_coords, 80, 56);
    //     assertEquals(4, res.size());
    //     // System.out.println(camerapose);
    //     // System.out.println(res.size() + " tags found");

    //     camerapose = new Pose3d(
    //         new Translation3d(4.0, 5.547868, 1.368552),
    //         new Rotation3d(0,0,0)
    //     );
    //     res = CameraTransforms.CalculateVisibleCoordinates(camerapose, tag_coords, 80, 56);
    //     // special case since the target point should never leave the dead center of the image frame
    //     assertEquals(0, res.size());

    //     // -0.0381, 5.547868, 1.368552 = speaker tag
    //     // 1.368552 / tan(20 deg) = 3.760066
    //     camerapose = new Pose3d( // The distance 3.765 was choosen to be just farther than the 20 degree distance from the point
    //         new Translation3d(3.765-0.0381, 5.547868, 0),
    //         new Rotation3d(0,0,Math.PI)
    //     );
    //     res = CameraTransforms.CalculateVisibleCoordinates(camerapose, tag_coords, 80, 40.0);
    //     assertEquals(4, res.size());
    //     assertEquals(0, res.get(0).get(0, 0), DELTA);
    //     assertEquals(1, res.get(0).get(1, 0), 0.01); // within 1% of the edge of the image is good enough

    //     camerapose = new Pose3d(
    //         new Translation3d(4.0, 5.547868, 1.368552),
    //         new Rotation3d(0,0,Math.PI+Units.degreesToRadians(39.9)) // Note that this is just barely less than half the HFOV
    //     );
    //     res = CameraTransforms.CalculateVisibleCoordinates(camerapose, tag_coords, 80, 56);
    //     assertEquals(4, res.size());
    //     assertEquals(1, res.get(0).get(0, 0), 0.01); // within 1% of the edge of the image is good enough
    //     assertEquals(0, res.get(0).get(1, 0), DELTA);
    // }

    // @Test
    // public void testLoadingFmap() {
    //     String fmap_path = "frc2024.fmap";
    //     var tags = FieldData.LoadTagLocations(fmap_path);
    //     assertEquals(16, tags.size());
    // }

    // @Test
    // public void testAllTagsInView() {
    //     double field_up_meters = 8.2;
    //     var camerapose = new Pose3d(
    //         new Translation3d(22.0, field_up_meters/2, 1.451102), // z is the speaker tag height
    //         new Rotation3d(0,0,Math.PI)
    //     );
    //     String fmap_path = "frc2024.fmap";
    //     var fmap_data = FieldData.LoadTagLocations(fmap_path);
    //     Quad[] tags = new Quad[fmap_data.size()];
    //     fmap_data.toArray(tags);
    //     var res = CameraTransforms.CalculateVisibleCoordinates(camerapose, tags, 80, 56, 0.2, 100);
    //     String desmos = "";
    //     for (var p : res) {
    //         desmos += "(" + p.get(0, 0) + "," + p.get(1, 0) + "),";
    //     }
    //     System.out.println(desmos); // uncomment to copy the above debug info into desmos to plot
    //     assertEquals(64, res.size());
    // }

    // @Test
    // public void testCropSpace() {
    //     var speaker_tag = new Translation3d(-0.0381, 5.547868, 1.368552);
    //     var tag_coords = new Quad[] {new Quad(speaker_tag)};

    //     var camerapose = new Pose3d(
    //         new Translation3d(5.0, 5.547868, 1.368552),
    //         new Rotation3d(0,0,Math.PI)
    //     );
    //     var res = CameraTransforms.CalculateVisibleCoordinates(camerapose, tag_coords, 56, 80);
    //     Translation2d margin = new Translation2d(0.2, 0.2);
    //     double[] bounds = CameraTransforms.FindBounds(res, margin);
    //     assertEquals(4, bounds.length);
    //     assertEquals(-1, bounds[0], DELTA); // llx
    //     assertEquals(-margin.getY(), bounds[1], DELTA); // lly
    //     assertEquals(1, bounds[2], DELTA); // urx
    //     assertEquals(margin.getY(), bounds[3], DELTA); // ury
    // }
}
