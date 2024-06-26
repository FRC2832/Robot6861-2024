package org.livoniawarriors;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.nio.charset.StandardCharsets;
import java.text.SimpleDateFormat;
import java.util.Date;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;

public class GitVersion implements Serializable {
    static final long serialVersionUID = 12345;
    public String lastCommit;
    public boolean isModified;
    public Date buildDate;
    public String buildAuthor;

    public void printVersions() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("SW Version");
        table.getEntry("Build Date").setString(new SimpleDateFormat("MM/dd/yyyy HH:mm:ss").format(buildDate));
        table.getEntry("Build Author").setString(buildAuthor);
        table.getEntry("Current Commit").setString(lastCommit);
        table.getEntry("Modified").setBoolean(isModified);
    }

    public static GitVersion loadVersion() {
        String path = Filesystem.getDeployDirectory() + "/gitinfo.obj";
        GitVersion obj;

        try {
            FileInputStream fileInputStream = new FileInputStream(path);
            ObjectInputStream objectInputStream = new ObjectInputStream(fileInputStream);
            obj = (GitVersion) objectInputStream.readObject();
            objectInputStream.close();
        } catch (Exception e) {
            // generic catch is usually bad, but here we are using it to create a default
            // whenever there is an issue loading it
            obj = new GitVersion();
            obj.buildDate = new Date();
            obj.isModified = false;
            obj.buildAuthor = "Unknown";
            obj.lastCommit = "Unknown";
        }
        return obj;
    }

    // this main function should only be called from Gradle
    public static void main(String[] args) {
        try {
            // setup the commands to run
            GitVersion result = new GitVersion();
            Runtime rt = Runtime.getRuntime();
            Process pr;
            result.buildDate = new Date();

            // get the user who made the commit
            pr = rt.exec("git config user.name");
            pr.waitFor();
            result.buildAuthor = new String(pr.getInputStream().readAllBytes(), StandardCharsets.UTF_8);
            // remove newline at end of name
            result.buildAuthor = result.buildAuthor.substring(0, result.buildAuthor.length() - 1);

            // run git log to get the last commits hash
            pr = rt.exec("git log -1 --pretty=tformat:%h");
            pr.waitFor();
            result.lastCommit = new String(pr.getInputStream().readAllBytes(), StandardCharsets.UTF_8);
            if (result.lastCommit.length() > 7) {
                result.lastCommit = result.lastCommit.substring(0, 7);
            }

            // get the status to see if any files have changed
            pr = rt.exec("git status -s");
            String temp = new String(pr.getInputStream().readAllBytes(), StandardCharsets.UTF_8);
            result.isModified = temp.length() != 0;

            // write object file
            FileOutputStream fileOutputStream = new FileOutputStream("src/main/deploy/gitinfo.obj");
            ObjectOutputStream objectOutputStream = new ObjectOutputStream(fileOutputStream);
            objectOutputStream.writeObject(result);
            objectOutputStream.flush();
            objectOutputStream.close();

        } catch (IOException e) {
            System.exit(1);
        } catch (InterruptedException e) {
            System.exit(1);
        }

        System.exit(0);
    }
}
