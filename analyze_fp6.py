import csv, glob, math, statistics as stats, os, re, sys

def best_per_frame_stats(path, matches_col, ttc_col):
    rows=[]
    with open(path) as f:
        r=csv.DictReader(f)
        for row in r:
            row={k.strip():v for k,v in row.items()}
            rows.append(row)
    by_frame={}
    for row in rows:
        fr=int(row["frame"])
        nm=int(float(row[matches_col]))
        ttc=row[ttc_col]
        ttc=float(ttc) if ttc not in ("nan","inf","-inf","") else math.nan
        if fr not in by_frame or nm>by_frame[fr]["nm"]:
            by_frame[fr]={"nm":nm,"ttc":ttc}
    vals=[v["ttc"] for v in by_frame.values() if not (math.isnan(v["ttc"]) or math.isinf(v["ttc"]))]
    vals=[x for x in vals if 0.5<x<100]
    if not vals:
        return 0, math.nan, math.nan, math.nan
    return len(vals), sum(vals)/len(vals), stats.median(vals), (stats.pstdev(vals) if len(vals)>1 else 0.0)

def combo_label(p):
    m=re.search(r'ttc_(camera|lidar)_(.+)\.csv$', p)
    return (m.group(1).upper(), m.group(2)) if m else ("?", os.path.basename(p))

# Console table
print(f"{'Combo':28}  {'Mod':4}  {'N':>3}  {'Mean':>7}  {'Median':>7}  {'StdDev':>7}")
print("-"*65)

rows_out=[]
for cam in sorted(glob.glob("results/ttc_camera_*.csv")):
    mod, combo = combo_label(cam)
    n,mean,med,std = best_per_frame_stats(cam, "nMatches", "TTC_camera")
    print(f"{combo:28}  CAM   {n:3d}  {mean:7.2f}  {med:7.2f}  {std:7.2f}")
    rows_out.append({"combo":combo,"mod":"CAM","N":n,"mean":mean,"median":med,"stddev":std})
    lid=f"results/ttc_lidar_{combo}.csv"
    if os.path.exists(lid):
        n,mean,med,std = best_per_frame_stats(lid, "nLidarCurr", "TTC_lidar")
        print(f"{combo:28}  LID   {n:3d}  {mean:7.2f}  {med:7.2f}  {std:7.2f}")
        rows_out.append({"combo":combo,"mod":"LID","N":n,"mean":mean,"median":med,"stddev":std})

# CSV export
os.makedirs("results", exist_ok=True)
out_path="results/fp6_summary.csv"
with open(out_path, "w", newline="") as f:
    w=csv.DictWriter(f, fieldnames=["combo","mod","N","mean","median","stddev"])
    w.writeheader()
    w.writerows(rows_out)
print(f"\nSaved CSV summary to: {out_path}")
