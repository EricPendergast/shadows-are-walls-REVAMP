using System.Collections.Generic;

class MultiDict<TKey, TValue>
{
   private Dictionary<TKey, List<TValue>> data =  new Dictionary<TKey,List<TValue>>();

   public void Add(TKey k, TValue v)
   {
      // can be a optimized a little with TryGetValue, this is for clarity
      if (data.ContainsKey(k))
         data[k].Add(v);
      else
        data.Add(k, new List<TValue>() {v}) ;
   }

   public List<TValue> Get(TKey k) {
       return data[k];
   }

   public void Clear() {
       data.Clear();
   }

   public Dictionary<TKey, List<TValue>>.Enumerator GetEnumerator() {
       return data.GetEnumerator();
   }
}
