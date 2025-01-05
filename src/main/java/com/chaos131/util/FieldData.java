package com.chaos131.util;

import com.chaos131.vision.AprilTag;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;

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
  public static ArrayList<Quad> LoadTagLocationsFromFile(String fmap_name) {
    Path fmap_path = Path.of(Filesystem.getDeployDirectory() + File.separator + fmap_name);
    String file_contents;
    try {
      file_contents = Files.readString(fmap_path);
      return LoadTagLocations(file_contents);
    } catch (IOException e) {
      return new ArrayList<Quad>();
    }
  }

  /**
   * Parses an FMAP's contents and produces April Tag data
   *
   * @param json the json in string format
   * @return the list of every April Tag in the fmap
   */
  public static ArrayList<Quad> LoadTagLocations(String json) {
    ArrayList<Quad> quads = new ArrayList<>();
    try {
      JsonNode productNode = new ObjectMapper().readTree(json);
      for (var fidu : productNode.get("fiducials")) {
        // if (fidu != null) System.out.println(fidu);

        double tag_size = fidu.get("size").asDouble();
        double[] transform =
            new ObjectMapper().readValue(fidu.get("transform").toString(), double[].class);
        var trans_mat = MatBuilder.fill(Nat.N4(), Nat.N4(), transform);

        var tag = new AprilTag(fidu.get("id").asInt(), trans_mat, tag_size / 1000.0);
        quads.add(tag);
      }
    } catch (IOException e) {
      // Probably a file not found issue, but maybe...?
      e.printStackTrace();
    }

    return quads;
  }

  private static ArrayList<Quad> AprilTags;

  /**
   * @return all the found april tags from a loaded file, null if no file has been loaded yet
   */
  public static ArrayList<Quad> getAllAprilTags() {
    if (AprilTags == null) {
      AprilTags = LoadTagLocationsFromFile("frc2024.fmap");
    }
    return AprilTags;
  }
}
