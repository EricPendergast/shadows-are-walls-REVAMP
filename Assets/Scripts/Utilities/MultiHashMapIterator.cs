using Unity.Collections;

namespace Utilities {
    public static partial class It {
        public static MultiHashMapIterator<U,T> Iterate<U,T>(NativeMultiHashMap<U, T> hashMap, U key) where T : struct
        where U : struct, System.IEquatable<U>{
            return new MultiHashMapIterator<U,T>(hashMap, key);
        }
    }
    // Iterates over one bucket in a multi hash map.
    public struct MultiHashMapIterator<U, T> 
        where T : struct
        where U : struct, System.IEquatable<U>
    {
        private NativeMultiHashMap<U, T> hashMap;
        NativeMultiHashMapIterator<U> it;
        private bool found;
        public T current;
        private T nextItem;
        public MultiHashMapIterator<U, T> GetEnumerator() => this;

        public MultiHashMapIterator(NativeMultiHashMap<U, T> hashMap, U key) {
            this.hashMap = hashMap;

            found = hashMap.TryGetFirstValue(key, out nextItem, out it);

            current = default(T);
        }

        public bool MoveNext() {
            if (!found) {
                return false;
            }

            current = nextItem;
            found = hashMap.TryGetNextValue(out nextItem, ref it);
            return true;
        }

        public T Current => current;
    }
}
