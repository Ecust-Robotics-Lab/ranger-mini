#!/usr/bin/env python3
"""Export the single-visit comparison; estimated references are labeled explicitly."""
import json
from pathlib import Path
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

root=Path(sys.argv[1]);output=Path(sys.argv[2])
retrieval=json.loads((root/'retrieval_2f_indoor_summary.json').read_text())['splits']['heldout_same_visit']
geometry=json.loads((root/'geometry_2f_indoor_summary.json').read_text())['splits']['heldout_same_visit']
methods=['v1','v2','sc20','sc80','std'];x=np.arange(len(methods));width=.26
fig,ax=plt.subplots(figsize=(9,4.8))
for offset,key,title,color in [(-width,'reference_region_hits_at_1','Reference region @1','#7293b6'),
                               (0,'reference_region_hits_at_5','Reference region @5','#315c85')]:
    values=[retrieval[m][key] for m in methods]
    bars=ax.bar(x+offset,values,width,label=title,color=color);ax.bar_label(bars,padding=3,fontsize=9)
values=[geometry[m]['queries_with_any_geometric_match'] for m in methods]
bars=ax.bar(x+width,values,width,label='Any KISS + ICP geometric match',color='#d39a40');ax.bar_label(bars,padding=3,fontsize=9)
ax.set_xticks(x,['Self v1','Self v2','SC 20 m','SC 80 m','STD indoor'])
ax.set_ylim(0,122);ax.set_ylabel('Queries / 107');ax.set_title('Held-out segment of one visit: 2-scan query, 4,000 points',pad=38)
ax.legend(loc='lower left',bbox_to_anchor=(0,1.01),ncol=3,fontsize=8);ax.spines[['right','top']].set_visible(False)
fig.text(.02,.015,'Position reference is estimated from baseline localization, not external ground truth.\nGeometric matches are candidate checks, not complete C2 initialization successes.',fontsize=8)
fig.tight_layout(rect=(0,.07,1,1));fig.savefig(output,dpi=200);plt.close(fig)
