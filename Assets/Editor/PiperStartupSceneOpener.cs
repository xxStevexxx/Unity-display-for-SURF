using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine.SceneManagement;

[InitializeOnLoad]
public static class PiperStartupSceneOpener
{
    private const string ScenePath = "Assets/Scenes/SampleScene.unity";
    private const string SessionKey = "PiperCubeGraspSimulation.OpenedStartupScene";

    static PiperStartupSceneOpener()
    {
        EditorApplication.delayCall += OpenStartupSceneIfNeeded;
    }

    [MenuItem("Piper/Open Cube Grasp Scene")]
    public static void OpenCubeGraspScene()
    {
        EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        SessionState.SetBool(SessionKey, true);
    }

    private static void OpenStartupSceneIfNeeded()
    {
        if (SessionState.GetBool(SessionKey, false))
            return;
        if (EditorApplication.isPlayingOrWillChangePlaymode)
            return;

        Scene activeScene = SceneManager.GetActiveScene();
        bool hasScenePath = !string.IsNullOrEmpty(activeScene.path);
        bool hasUnsavedChanges = activeScene.isDirty;
        if (hasScenePath || hasUnsavedChanges)
        {
            SessionState.SetBool(SessionKey, true);
            return;
        }

        EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        SessionState.SetBool(SessionKey, true);
    }
}
