"""Build journal-neutral PDFs and editable LaTeX from shared content blocks."""
from collections import Counter
from datetime import date
import html
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image as PILImage
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Image, Table, TableStyle, KeepTogether, PageBreak

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from manuscript_content import AUTHOR, TITLE, REFERENCES, manuscript

PDF=HERE/'output'/'pdf';TEX=HERE/'output'/'latex';QA=HERE/'_qa';WIDTH=A4[0]-108
STATES=('recovered','unsettled','fallen')


def technical(data,audit,original):
    b=[]
    def h(t):b.append(('heading',t))
    def p(t):b.append(('paragraph',t))
    h('Purpose and main findings')
    p('This technical report documents the numerical evidence supporting the accompanying manuscript. It preserves the complete torque-slice atlas, exact outcome counts, paired comparisons, controller settings, verification checks, and the distinction between completed and proposed experiments. It is intended to support independent review and reproduction.')
    p('The original study compares two fixed implementations on a nonlinear planar model. PID-type remains upright in 435 of 819 conditions but recovers position in only 215. LQR recovers in all 357 conditions where it stays upright. The 220 non-recovering PID-type survivors approach integral-dependent position offsets. Extending those cases to 30 s does not change their labels. Removing only the integral gain converts all 220 to recovery and adds two falls elsewhere.')
    b.append(('table','Outcome summary',['Implementation','Recovered','Unsettled','Fallen','Total'],[['Original PID-type',215,220,384,819],['Original LQR',357,0,462,819],['PID-type with ki zero',433,0,386,819]],[.36,.16,.16,.16,.16]))
    p('The ablation and horizon extension are post hoc analyses. They explain this implementation and do not establish a class-wide ranking. Counts are properties of a fixed parameter grid; no random sampling distribution, significance test, or binomial confidence interval is implied.')
    h('Study inventory')
    b.append(('table','Completed numerical work',['Analysis','Evaluations','Purpose'],[['Original grid','1638','Two fixed controllers on 819 shared conditions'],['Exact baseline reproduction','1638','All stored metric values verified at saved precision'],['Horizon extension','220','All original unsettled cases continued to 30 s'],['Integral ablation','819','ki changed from 0.4 to zero only'],['Refined integration',audit['audited'],'All boundary-adjacent cases at half physics step'],['Tolerance sensitivity','Reclassification','Final-window bounds applied to saved trajectories']],[.32,.18,.50]))
    p('Baseline reproduction repeats the original trials and must not be counted as independent evidence. The refined integration audit repeats selected conditions with a changed numerical solver resolution. No additional physical robots, seeds, or sensor-noise realizations are implied by these totals.')
    h('Original outcome counts by torque limit')
    rows=[]
    for torque in original['metadata']['axes']['torque_limit_nm']:
        for name in ('pid','lqr'):
            selected=[r for r in data['baseline'] if r['controller']==name and r['torque_limit_nm']==torque]
            counts=Counter(r['outcome'] for r in selected)
            rows.append([f'{torque:g}',name.upper(),*[counts[s] for s in STATES],sum(r['fallen'] and r['push_force_n']!=0 and not r['push_started'] for r in selected)])
    b.append(('table','117 conditions per controller and torque limit',['Limit N m','Controller','R','U','F','Pre-push F'],rows,[.17,.23,.12,.12,.12,.24]))
    p('R is recovered, U is upright but unsettled, and F is fallen. Pre-push F is a subset of F for trials with a planned nonzero push; the robot fell before any pulse was applied. Multiple planned push values can therefore correspond to the same initial-tilt failure. All such cases are retained in the stated Cartesian grid rather than silently removed.')
    b.append(('pagebreak',))
    h('Reproduction and numerical definitions')
    p('The nominal model uses body mass 1.2 kg, translating base mass 0.35 kg, axle-to-body-centre distance 0.18 m, body-centre inertia 0.012 kg m², wheel radius 0.045 m, and each wheel inertia 0.00012 kg m². Translation and rotation drag are 0.08 N s/m and 0.002 N m s/rad. Gravity is 9.81 m/s². These values were not identified from hardware.')
    p('State order is x, v, theta, omega, with displacement in metres and angle in radians. The input is the total torque of both wheel motors. A force at the body centre enters translation and rotation. Gains are fixed across torque limits. The PID-type controller uses kp = 14, kd = 2, kx = 2, kv = 3, ki = 0.4, an integral clamp of 0.25 rad s, and conditional anti-windup. LQR uses Q = diag(12,2,120,2), R = 1 and the gain vector supplied in the archived configuration.')
    p('Each baseline run lasts at most 10 s, with a 0.2 s horizontal force pulse beginning at 2 s. Controls are held for 5 ms. RK4 integrates the plant. Fall detection is evaluated at control endpoints, at 45 degrees absolute tilt. Recovery requires a final dwell of at least one second within 1 degree tilt, 3 cm position, and 3 cm/s speed. For a nonzero pulse, recovery timing begins at 2.2 s; otherwise it begins at zero.')
    h('Linearization and integral state')
    b.append(('table','PID-type augmented continuous poles',['Real part','Imaginary part'],[[f'{r:.8g}',f'{i:.8g}'] for r,i in data['pid_linear_poles']],[.5,.5]))
    p('The near-zero augmented mode corresponds to the equilibrium family x = -0.2 z. For the 220 continued cases, the largest 30 s difference between this predicted position and the unrounded numerical state is 2.76e-12 m. This is a consistency check inside the model, not a physical accuracy estimate. Stored rounded endpoint values in CSV are deliberately less precise than the raw state used for that residual.')
    h('Extended integration audit')
    p(f'All {audit["audited"]} cells adjacent to a different outcome along tilt, force, or torque were recomputed with two physics substeps per 5 ms control interval. No controller update or event-detection interval was changed. There were {audit["disagreements"]} changes in the three-way outcome label. The maximum change in peak tilt was {max(r["peak_tilt_difference_deg"] for r in audit["records"]):.6g} degrees. The complete audit records are included in data/full_integration_audit.json.')
    p('This is stronger than the original sampled 32-cell check, but it is still a finite numerical audit. It is not a formal error bound, an interval-arithmetic certificate, or proof of stability between grid points. Hardware-model mismatch is unaffected by refining the integrator.')
    h('Tests and provenance')
    p('The baseline tests cover upright equilibrium, mechanical power balance, unforced energy conservation, RK4 convergence, parameter validation, local sampled LQR stability, nominal recovery, falls, anti-windup, and Python-to-JavaScript agreement. Sweep tests cover completeness, repeatability, nonmonotonic neighbouring outcomes, pre-push falls, report preservation, and fixed control timing under integration refinement. UI logic tests use mocked DOM and canvas objects; they are not a substitute for visual browser inspection.')
    p('The publication analysis additionally asserts equality of every stored baseline metric during reproduction and verifies that reclassification at the original 3 cm tolerance returns the original recovery counts. Analysis source hashes, the original result-file hash, configuration, and software versions are included in the package. Original model parameters and gains are preserved; the ablation is a separate configuration copy.')
    h('Paired recovering subset')
    paired=[p for p in data['paired'] if p['pid']['outcome']==p['lqr']['outcome']=='recovered']
    rows=[]
    for key,label in [('recovery_s','Recovery time s'),('peak_tilt_deg','Peak tilt deg'),('max_displacement_m','Max displacement m'),('torque_squared_integral','Squared torque integral')]:
        rows.append([label,f'{np.median([p["pid"][key] for p in paired]):.6g}',f'{np.median([p["lqr"][key] for p in paired]):.6g}',f'{np.median([p["lqr"][key]-p["pid"][key] for p in paired]):.6g}'])
    b.append(('table',f'Medians on {len(paired)} common recoveries',['Metric','PID-type','LQR','Paired LQR minus PID'],rows,[.34,.18,.18,.30]))
    p('The final column is the median of paired differences, not the difference of two medians. Every condition that falls or fails recovery under either controller is excluded from this table. The exclusion must accompany any interpretation of response speed or effort.')
    h('Three-dimensional visualisations')
    p('The companion visualisation/index.html displays six saved trajectories as orbitable three-dimensional robot schematics. Displacement, pitch and force timing are copied from the original trajectories. Wheel rotation uses the no-slip constraint. Display lanes separate the controllers; there is no lateral or yaw dynamics. The shell and track width are illustrative. A frozen final pose marks a fall rather than a simulated ground impact.')
    p('visualisation/experiments.html displays all 1,638 baseline records as discrete points in initial-tilt, push-force and torque-limit space, with controller, outcome and torque-slice filters. It does not interpolate a stability surface. The static figure below displays selected paths in state space; the animated GIF supplies a browser-independent robot preview. Interactive WebGL rendering could not be visually inspected because the browser access host was unavailable. Automated checks verify exact replay data, sample selection, centre-of-mass mapping, force timing and outcome totals.')
    b.append(('figure','fig09_3d_state_trajectories'))
    for i,torque in enumerate(original['metadata']['axes']['torque_limit_nm'],1):
        b.append(('pagebreak',))
        if i==1:
            h('Complete outcome atlas')
            p('These seven torque slices show every original condition. Axes and symbol meanings are identical across slices. No interpolation or fitted boundary is used. A listed recovered force at zero initial tilt is an evaluated value only; it is not a guaranteed disturbance threshold.')
        h(f'Torque slice {torque:g} N m')
        b.append(('figure',f'sfig{i:02d}_torque_{str(torque).replace(".","p")}'))
        rows=[]
        for controller in ('pid','lqr'):
            selected=[r for r in data['baseline'] if r['controller']==controller and r['torque_limit_nm']==torque]
            counts=Counter(r['outcome'] for r in selected)
            rows.append([controller.upper(),*[counts[s] for s in STATES],max(r['max_displacement_m'] for r in selected)])
            recovered=[r['push_force_n'] for r in selected if r['initial_tilt_deg']==0 and r['outcome']=='recovered']
            p(f'{controller.upper()} recovered at zero initial tilt for these tested force values: '+(', '.join(f'{v:g} N' for v in recovered) if recovered else 'none')+'.')
        b.append(('table','Slice summary',['Controller','R','U','F','Largest travel m'],[[*r[:-1],f'{r[-1]:.3f}'] for r in rows],[.28,.12,.12,.12,.36]))
        p('Every slice contains 117 conditions for each controller. Larger travel is not penalized directly by the fall label; the operational position criterion applies to the final dwell. Conditions with early falls can be duplicated across planned forces because the pulse is scheduled after the initial balancing transient.')
    b.append(('pagebreak',));h('Steps 3 and 4 and the submission pathway')
    p('The current material is a journal-neutral manuscript draft and a reproducible simulation package. It has not established a new general control method, hardware validity, or superiority after matched tuning. Before selecting a submission venue, the author should decide whether the intended contribution is an evaluation methodology, a new estimator or controller, or a validated robotics implementation. Additional features alone do not establish novelty.')
    h('Step 3 Imperfect measurements')
    p('Begin with a measurement model defined independently from the true simulated state. Introduce angular measurement noise and gyro noise separately, then introduce observation delay through an explicit history buffer. Keep force timing and controller period fixed. Label a simple angular-noise surrogate as such; a physical accelerometer model must also account for translational acceleration and orientation. A complementary or Kalman estimator should use only past and present measurements.')
    p('Select conditions near and away from the present boundaries before examining noisy outcomes. Use common random-number seeds across controllers where the disturbance model permits, and preserve individual trials. An initial design could use at least 30 independent noise realizations per stochastic condition, with more trials where interval width remains large. Report fall frequency with an appropriate binomial interval for independent repeats of a fixed condition, plus recovery and estimator error. Do not apply those intervals to the present deterministic grid.')
    h('Step 4 Physically consistent payload variation')
    p('Start with static payloads present at initialization. For a payload mass mp at vertical distance lp from the axle, update total body mass to m + mp and centre distance to (m l + mp lp)/(m + mp). Recompute body inertia with the parallel-axis theorem, including the payload inertia. Lateral offsets require a changed equilibrium orientation and a correspondingly consistent gravitational model; they cannot be represented by changing mass alone.')
    p('First hold the original gains fixed to characterize robustness. Compare retuned or adaptive controllers only in a separate, transparently specified experiment. Distinguish calibration conditions from held-out tests. A sudden load pickup requires an explicit treatment of the initial state and momentum exchange; it is not equivalent to silently replacing parameters during an integration step. Only after separate sensing and payload studies should their interaction be tested.')
    h('Work needed before submission')
    b.append(('table','Development priorities',['Priority','Deliverable','Evidence required'],[
      ['1','Fair baseline comparison','Matched tuning objective or budget, several gain sets, and held-out conditions'],
      ['2','Independent model validation','Cross-check with another simulator or identified hardware response'],
      ['3','Sensing and delay study','Explicit measurement model, estimator, seeds, uncertainty summaries'],
      ['4','Payload study','Consistent mass properties, frozen-gain and redesigned-gain comparisons'],
      ['5','Contribution and related work','A narrow novelty statement supported by a deeper literature review'],
      ['6','Reproducible release','Reviewed code, frozen environment, data archive, and persistent identifier']],[.12,.32,.56]))
    p('No external submission, repository publication, data upload, or claim of peer review has been made. Michael Agbo should review and approve the scientific content, authorship details, affiliation, declarations, and final venue requirements. AI-assisted coding and drafting should be disclosed according to the selected venue policy.')
    h('References')
    for i,(_,citation,url) in enumerate(REFERENCES,1):b.append(('reference',f'[{i}] {citation}',url))
    return b


def setup_fonts():
    for name,file in [('Body','times.ttf'),('BodyBold','timesbd.ttf'),('BodyItalic','timesi.ttf'),('Sans','arial.ttf'),('SansBold','arialbd.ttf')]:
        pdfmetrics.registerFont(TTFont(name,str(Path('C:/Windows/Fonts')/file)))
    pdfmetrics.registerFontFamily('Body',normal='Body',bold='BodyBold',italic='BodyItalic',boldItalic='BodyBold')


def eq_image(latex,index):
    target=QA/f'equation_{index}.png'
    fig=plt.figure(figsize=(6.7,.42));fig.text(.5,.5,'$'+latex+'$',ha='center',va='center',fontsize=12)
    fig.savefig(target,dpi=400,bbox_inches='tight',pad_inches=.03,facecolor='white');plt.close(fig)
    return target


def build_pdf(target,title,blocks,manifest,subtitle):
    styles={
      'body':ParagraphStyle('body',fontName='Body',fontSize=11,leading=14.3,spaceAfter=8),
      'heading':ParagraphStyle('heading',fontName='SansBold',fontSize=12,leading=15,spaceBefore=13,spaceAfter=7,keepWithNext=True),
      'title':ParagraphStyle('title',fontName='BodyBold',fontSize=21,leading=25,spaceAfter=12),
      'author':ParagraphStyle('author',fontName='Body',fontSize=12,leading=16,spaceAfter=8),
      'caption':ParagraphStyle('caption',fontName='Body',fontSize=9.3,leading=12,spaceBefore=5,spaceAfter=12),
      'tablecaption':ParagraphStyle('tablecaption',fontName='Body',fontSize=9.3,leading=12,spaceBefore=5,spaceAfter=8,keepWithNext=True),
      'table':ParagraphStyle('table',fontName='Body',fontSize=9.3,leading=12),
      'tablehead':ParagraphStyle('tablehead',fontName='SansBold',fontSize=9,leading=11),
      'reference':ParagraphStyle('reference',fontName='Body',fontSize=9.6,leading=12.5,spaceAfter=8),
    }
    story=[Paragraph(html.escape(title),styles['title']),Paragraph(AUTHOR,styles['author']),Paragraph(html.escape(subtitle),styles['caption'])]
    number=0;eqcount=0
    for block in blocks:
        kind=block[0]
        if kind=='heading' and block[1]=='References':
            refs=[Paragraph(html.escape(b[1])+'<br/><link href="'+html.escape(b[2],quote=True)+'" color="#000000">'+html.escape(b[2])+'</link>',styles['reference']) for b in blocks if b[0]=='reference']
            story.append(KeepTogether([Paragraph('References',styles['heading']),*refs]))
        elif kind in ('heading','paragraph'):story.append(Paragraph(html.escape(block[1]),styles['heading' if kind=='heading' else 'body']))
        elif kind=='pagebreak':story.append(PageBreak())
        elif kind=='reference':pass
        elif kind=='equation':
            eqcount+=1;file=eq_image(block[1],str(target.stem)+'_'+str(eqcount));im=PILImage.open(file);width=min(WIDTH,im.width*72/400);height=width*im.height/im.width
            story.append(KeepTogether([Spacer(1,2),Image(str(file),width=width,height=height),Spacer(1,7)]))
        elif kind=='figure':
            number+=1;entry=manifest[block[1]];file=HERE/'figures'/f'{block[1]}.png';im=PILImage.open(file);width=min(WIDTH,570*im.width/im.height);height=width*im.height/im.width
            caption=f'Figure {number}. '+entry['caption']
            story.append(KeepTogether([Image(str(file),width=width,height=height),Paragraph(html.escape(caption),styles['caption'])]))
        elif kind=='table':
            _,title,headers,rows,fractions=block
            cells=[[Paragraph(html.escape(str(x)),styles['tablehead']) for x in headers]]+[[Paragraph(html.escape(str(x)),styles['table']) for x in row] for row in rows]
            table=Table(cells,colWidths=[WIDTH*x for x in fractions],repeatRows=1,hAlign='LEFT')
            table.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#e8edf2')),('GRID',(0,0),(-1,-1),.4,colors.HexColor('#d0d0d0')),('VALIGN',(0,0),(-1,-1),'MIDDLE'),('LEFTPADDING',(0,0),(-1,-1),7),('RIGHTPADDING',(0,0),(-1,-1),7),('TOPPADDING',(0,0),(-1,-1),6),('BOTTOMPADDING',(0,0),(-1,-1),6)]))
            story.extend([Paragraph(html.escape(title),styles['tablecaption']),table,Spacer(1,10)])
    def page(canvas,doc):
        canvas.saveState();canvas.setFont('Body',9);canvas.setFillColor(colors.HexColor('#555555'));canvas.drawString(54,30,'Michael Agbo');canvas.drawRightString(A4[0]-54,30,str(doc.page));canvas.restoreState()
    doc=SimpleDocTemplate(str(target),pagesize=A4,leftMargin=54,rightMargin=54,topMargin=48,bottomMargin=48,title=title,author=AUTHOR)
    doc.build(story,onFirstPage=page,onLaterPages=page)


def escape_tex(text):
    substitutions={'\\':r'\textbackslash{}','&':r'\&','%':r'\%','$':r'\$','#':r'\#','_':r'\_','{':r'\{','}':r'\}','~':r'\textasciitilde{}','^':r'\textasciicircum{}','²':r'\textsuperscript{2}','×':r'\ensuremath{\times}','±':r'\ensuremath{\pm}','°':r'\ensuremath{^\circ}'}
    return ''.join(substitutions.get(c,c) for c in str(text))


def build_tex(target,title,blocks,manifest):
    parts=[r'\documentclass[11pt,a4paper]{article}',r'\usepackage[utf8]{inputenc}',r'\usepackage[T1]{fontenc}',r'\usepackage[margin=19mm]{geometry}',r'\usepackage{amsmath,graphicx,booktabs,array,longtable,caption}',r'\usepackage[hidelinks]{hyperref}',r'\setlength{\tabcolsep}{3pt}',r'\setlength{\parindent}{0pt}',r'\setlength{\parskip}{6pt}',r'\title{'+escape_tex(title)+'}',r'\author{'+AUTHOR+'}',r'\date{12 November 2026}',r'\begin{document}',r'\maketitle']
    for block in blocks:
        kind=block[0]
        if kind=='heading':parts.append(r'\section*{'+escape_tex(block[1])+'}')
        elif kind=='paragraph':parts.append(escape_tex(block[1])+'\n')
        elif kind=='equation':parts.append(r'\begin{equation}'+block[1]+r'\end{equation}')
        elif kind=='pagebreak':parts.append(r'\clearpage')
        elif kind=='figure':parts.extend([r'\begin{figure}[htbp]',r'\centering',r'\includegraphics[width=\linewidth,height=.65\textheight,keepaspectratio]{../../figures/'+block[1]+'.pdf}',r'\caption{'+escape_tex(manifest[block[1]]['caption'])+'}',r'\end{figure}'])
        elif kind=='reference':parts.append(escape_tex(block[1])+r' \url{'+block[2]+r'}\par')
        elif kind=='table':
            _,title,headers,rows,widths=block
            # Allow tabcolsep when allocating paragraph-column widths.
            columns=''.join('p{'+str(round(w*.9,3))+r'\linewidth}' for w in widths)
            parts.extend([r'\textbf{'+escape_tex(title)+'}',r'\begin{longtable}{'+columns+'}',r'\toprule',' & '.join(escape_tex(x) for x in headers)+r' \\ \midrule \endhead'])
            parts.extend(' & '.join(escape_tex(x) for x in row)+r' \\' for row in rows)
            parts.extend([r'\bottomrule',r'\end{longtable}'])
    parts.append(r'\end{document}');target.write_text('\n'.join(parts)+'\n',encoding='utf-8')


def main():
    for folder in (PDF,TEX,QA):folder.mkdir(parents=True,exist_ok=True)
    setup_fonts();data=json.loads((HERE/'data'/'analysis.json').read_text());audit=json.loads((HERE/'data'/'full_integration_audit.json').read_text());original=json.loads((HERE.parent/'results'/'robustness'/'results.json').read_text())
    manifest={r['name']:r for r in json.loads((HERE/'figure_manifest.json').read_text())}
    main_blocks=manuscript(data,audit);supp_blocks=technical(data,audit,original)
    for name,title,blocks,subtitle in [('manuscript',TITLE,main_blocks,'Nonlinear simulation study and controller ablation | 12 November 2026'),('technical_report','Technical report and complete balancing outcome atlas',supp_blocks,'Methods, numerical evidence, full maps and research development plan | 12 November 2026')]:
        build_pdf(PDF/f'{name}.pdf',title,blocks,manifest,subtitle)
        build_tex(TEX/f'{name}.tex',title,blocks,manifest)
        (HERE/'data'/f'{name}_content.json').write_text(json.dumps(blocks,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
        print('Built',PDF/f'{name}.pdf',flush=True)


if __name__=='__main__':main()
