using UnityEngine;

// Note: To use this, you must put the scriptable object instance into the
// Resources folder. You must also put the following lines in any class
// inheriting from this:
//  
//  [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
//  static void Init() {
//      SingletonInit();
//  }
//
// Add to asset menu with the following:
// [CreateAssetMenu(fileName = "XXXX", menuName = "ScriptableObjects/XXXX")]
public class SingletonScriptableObject<T> : ScriptableObject where T : ScriptableObject {

    private static T instance;
    public static T Instance { get => instance; }

    protected static void SingletonInit() {
        var instances = Resources.LoadAll<T>("");
        if (instances.Length != 1) {
            Debug.LogError("Error: " + instances.Length + " instances of class " + typeof(T).ToString() + ". Singleton will be set to null.");
            instance = null;
        } else {
            instance = instances[0];
        }
    }
}
