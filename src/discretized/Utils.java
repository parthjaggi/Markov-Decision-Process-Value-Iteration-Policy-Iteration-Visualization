package discretized;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

import com.google.gson.Gson;

public final class Utils {
	public static void saveToFile(java.lang.Object src, String filepath) {
		String content = new Gson().toJson(src);
		Utils.saveStringToFile(content, filepath);
	}

	public static void saveStringToFile(String content, String filepath) {
		try {
			BufferedWriter writer = new BufferedWriter(new FileWriter(filepath));
			writer.write(content);
			writer.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
}
