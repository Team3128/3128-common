// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package common.hardware.camera;

import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.UncheckedIOException;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.HashMap;
import java.util.Iterator; 
import java.util.Map; 
  
import org.json.simple.parser.*; 
  

/** Loadable AprilTag field layouts. */
public enum AprilTagFields {
  /** 2022 Rapid React. */
  k2022RapidReact("2022-rapidreact.json"),
  /** 2023 Charged Up. */
  k2023ChargedUp("2023-chargedup.json"),
  /** 2024 Crescendo. */
  k2024Crescendo("2024-crescendo.json"),
  /** 2024 Offseason Crescendo */
  k2024OffseasonCrescendo("OffseasonAprilTags.json");

  /** Base resource directory. */
  public static final String kBaseResourceDir = "//common//hardware//camera//";

  /** Alias to the current game. */
  public static final AprilTagFields kDefaultField = k2024OffseasonCrescendo;

  /** Resource filename. */
  public final String m_resourceFile;

  AprilTagFields(String resourceFile) {
    m_resourceFile = kBaseResourceDir + resourceFile;
  }

  /**
   * Get a {@link AprilTagFieldLayout} from the resource JSON.
   *
   * @return AprilTagFieldLayout of the field
   * @throws UncheckedIOException If the layout does not exist
   */
  public AprilTagFieldLayout loadAprilTagLayoutField() {
    try {
      return loadFromResource(this.m_resourceFile);
    } catch (IOException e) {
      throw new UncheckedIOException(
          "Could not load AprilTagFieldLayout from " + this.m_resourceFile, e);
    }
  }
  

  public static AprilTagFieldLayout loadFromResource(String resourcePath) throws IOException {
    InputStream stream = AprilTagFieldLayout.class.getResourceAsStream(resourcePath);
    if (stream == null) {
      // Class.getResourceAsStream() returns null if the resource does not exist.
      throw new IOException("Could not locate resource: " + resourcePath);
    }
    InputStreamReader reader = new InputStreamReader(stream);
    try {
      return new ObjectMapper().readerFor(AprilTagFieldLayout.class).readValue(reader);
    } catch (IOException e) {
      throw new IOException("Failed to load AprilTagFieldLayout: " + resourcePath);
    }
  }


}
