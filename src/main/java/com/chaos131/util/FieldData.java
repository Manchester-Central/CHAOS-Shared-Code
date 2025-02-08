package com.chaos131.util;

import com.chaos131.vision.AprilTag;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;

/**
 * An empty class definition that should be fleshed out with functions to read fmap file contents
 */
public class FieldData {
  /**
   * Parses an FMAP file containing April Tag locations
   *
   * @param fmap_name typically just the filename, since it should read from the deploy folder by
   *     default
   * @return the list of every April Tag in the fmap
   */
  public static ArrayList<AprilTag> LoadTagLocationsFromFile(String fmap_name) {
    Path fmap_path = Path.of(Filesystem.getDeployDirectory() + File.separator + fmap_name);
    String file_contents;
    try {
      file_contents = Files.readString(fmap_path);
      return LoadTagLocations(file_contents);
    } catch (IOException e) {
      return new ArrayList<AprilTag>();
    }
  }

  /**
   * @param fmap_name file name to read from
   * @return HashMap using AprilTag IDs as the key and the tag data as the value
   */
  public static HashMap<Integer, AprilTag> GetAprilTagMap(String fmap_name) {
    HashMap<Integer, AprilTag> mapping = new HashMap<>();
    ArrayList<AprilTag> all_tags = LoadTagLocationsFromFile(fmap_name);
    for (var tag : all_tags) {
      mapping.put(tag.id, tag);
    }
    return mapping;
  }

  /**
   * @param tags array list of april tags
   * @return Pose2d array of all tags in the given ArrayList
   */
  public static Pose2d[] GatherAprilTagPoses(ArrayList<AprilTag> tags) {
    Pose2d[] poses = new Pose2d[tags.size()];
    for (int idx = 0; idx < tags.size(); idx++) {
      poses[idx] = tags.get(idx).pose2d;
    }
    return poses;
  }

  /**
   * Parses an FMAP's contents and produces April Tag data
   *
   * @param json the json in string format
   * @return the list of every April Tag in the fmap
   */
  public static ArrayList<AprilTag> LoadTagLocations(String json) {
    ArrayList<AprilTag> tags = new ArrayList<>();
    try {
      JsonNode productNode = new ObjectMapper().readTree(json);
      Transform3d coordinate_shift = new Transform3d();
      if (productNode.get("fieldlength") != null) {
        coordinate_shift =
            new Transform3d(
                productNode.get("fieldlength").asDouble() / 2.0,
                productNode.get("fieldwidth").asDouble() / 2.0,
                0,
                new Rotation3d());
      }
      for (var fidu : productNode.get("fiducials")) {
        // if (fidu != null) System.out.println(fidu);

        double tag_size = fidu.get("size").asDouble();
        double[] transform =
            new ObjectMapper().readValue(fidu.get("transform").toString(), double[].class);
        var trans_mat = MatBuilder.fill(Nat.N4(), Nat.N4(), transform);

        var tag =
            new AprilTag(fidu.get("id").asInt(), trans_mat, tag_size / 1000.0, coordinate_shift);
        tags.add(tag);
      }
    } catch (IOException e) {
      // Probably a file not found issue, but maybe...?
      e.printStackTrace();
    }

    return tags;
  }
}
