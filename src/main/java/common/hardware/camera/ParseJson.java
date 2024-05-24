package common.hardware.camera;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.HashMap;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class ParseJson {

    HashMap<Integer, Pose3d> offseasonTags = new HashMap<Integer, Pose3d>();
    public ParseJson(String filepath){

      Object obj = new Object();
      try {
        obj = new JSONParser().parse(new FileReader(filepath));
      } catch (FileNotFoundException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      } catch (IOException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      } catch (ParseException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      } 
        
      // typecasting obj to JSONObject 
      JSONObject jo = (JSONObject) obj; 
        
      JSONArray poses = ((JSONArray) jo.get("tags")); 
         

      for (int i = 0; i < poses.size(); i++) {
        JSONObject tag = (JSONObject) poses.get(i);
        JSONObject pose = (JSONObject) tag.get("pose");
        JSONObject translation = (JSONObject) pose.get("translation");
        JSONObject rotation = (JSONObject) pose.get("rotation");
        double rotationW = Double.parseDouble(rotation.get("W").toString());
        double rotationX = Double.parseDouble(rotation.get("X").toString());
        double rotationY = Double.parseDouble(rotation.get("Y").toString());
        double rotationZ = Double.parseDouble(rotation.get("Z").toString());
        Quaternion quaternion = new Quaternion(rotationW, rotationX, rotationY,rotationZ);
        double translationx = Double.parseDouble(translation.get("x").toString());
        double translationy = Double.parseDouble(translation.get("y").toString());
        double translationz = Double.parseDouble(translation.get("z").toString());
        Translation3d translation3d = new Translation3d(translationx, translationy, translationz);
        Pose3d apriltag = new Pose3d(translation3d, new Rotation3d(quaternion));
        // System.out.print(i + " " + translation.get("x"));
        offseasonTags.put(i, apriltag);
      }

    }

    public HashMap<Integer, Pose3d> returnHashMap(){
        return offseasonTags;
    }
}
