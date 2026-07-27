#!/usr/bin/env python3
"""Build the annotated Chinese manuscript draft from the official CEA template."""

from copy import deepcopy
from pathlib import Path

from docx import Document
from docx.enum.section import WD_SECTION
from docx.enum.table import WD_CELL_VERTICAL_ALIGNMENT, WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_BREAK
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Inches, Pt, RGBColor


ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / "docs"
TEMPLATE = DOCS / "计算机工程与应用_官方论文写作模板_20251103.docx"
OUTPUT = DOCS / "长走廊退化环境下激光雷达-里程计各向异性融合方法_投稿初稿_作者批注版.docx"

BLACK = RGBColor(0x00, 0x00, 0x00)
RED = RGBColor(0xC0, 0x00, 0x00)
BLUE = RGBColor(0x1F, 0x4E, 0x79)
GREEN = RGBColor(0x54, 0x82, 0x35)
PURPLE = RGBColor(0x70, 0x30, 0xA0)
GRAY = RGBColor(0x66, 0x66, 0x66)


def set_run_font(run, cn="宋体", en="Times New Roman", size=10, bold=None, italic=None):
    run.font.name = en
    run._element.get_or_add_rPr().rFonts.set(qn("w:eastAsia"), cn)
    run.font.size = Pt(size)
    if bold is not None:
        run.bold = bold
    if italic is not None:
        run.italic = italic


def set_style_font(style, cn="宋体", en="Times New Roman", size=10):
    style.font.name = en
    style._element.get_or_add_rPr().rFonts.set(qn("w:eastAsia"), cn)
    style.font.size = Pt(size)


def clear_document_body(doc):
    body = doc._element.body
    sect_pr = body.sectPr
    for child in list(body):
        if child is not sect_pr:
            body.remove(child)


def configure_styles(doc):
    for name in ["Normal", "Normal (Web)", "毕业论文正文"]:
        if name in doc.styles:
            set_style_font(doc.styles[name], size=10)
    for name, size in [("Heading 7", 11), ("Heading 8", 10), ("Heading 3", 10)]:
        if name in doc.styles:
            set_style_font(doc.styles[name], cn="黑体", size=size)
            doc.styles[name].font.bold = True
    for name, size in [("Subtitle", 18), ("Name", 14)]:
        if name in doc.styles:
            set_style_font(doc.styles[name], cn="黑体", size=size)
    if "参考文献" in doc.styles:
        set_style_font(doc.styles["参考文献"], size=8)


def set_columns(section, number=1, space_twips=425):
    sect_pr = section._sectPr
    cols = sect_pr.find(qn("w:cols"))
    if cols is None:
        cols = OxmlElement("w:cols")
        sect_pr.append(cols)
    if number <= 1:
        cols.attrib.pop(qn("w:num"), None)
    else:
        cols.set(qn("w:num"), str(number))
    cols.set(qn("w:space"), str(space_twips))


def configure_section(section, columns=1):
    section.page_width = Cm(19.18)
    section.page_height = Cm(26.25)
    section.top_margin = Cm(1.28)
    section.bottom_margin = Cm(0.8)
    section.left_margin = Cm(0.56)
    section.right_margin = Cm(0.56)
    set_columns(section, columns)


def add_run(paragraph, text, color=BLACK, bold=False, italic=False, size=10, cn="宋体"):
    run = paragraph.add_run(text)
    set_run_font(run, cn=cn, size=size, bold=bold, italic=italic)
    run.font.color.rgb = color
    return run


def add_body(doc, text, color=BLACK, bold=False, italic=False, indent=True, align=WD_ALIGN_PARAGRAPH.JUSTIFY):
    paragraph = doc.add_paragraph(style="Normal")
    paragraph.alignment = align
    paragraph.paragraph_format.line_spacing = 1.0
    paragraph.paragraph_format.space_after = Pt(0)
    paragraph.paragraph_format.space_before = Pt(0)
    if indent:
        paragraph.paragraph_format.first_line_indent = Pt(20)
    add_run(paragraph, text, color=color, bold=bold, italic=italic, size=10)
    return paragraph


def add_mixed(doc, chunks, indent=True, align=WD_ALIGN_PARAGRAPH.JUSTIFY):
    paragraph = doc.add_paragraph(style="Normal")
    paragraph.alignment = align
    paragraph.paragraph_format.line_spacing = 1.0
    paragraph.paragraph_format.space_after = Pt(0)
    if indent:
        paragraph.paragraph_format.first_line_indent = Pt(20)
    for chunk in chunks:
        text = chunk[0]
        color = chunk[1] if len(chunk) > 1 else BLACK
        bold = chunk[2] if len(chunk) > 2 else False
        italic = chunk[3] if len(chunk) > 3 else False
        add_run(paragraph, text, color=color, bold=bold, italic=italic)
    return paragraph


def add_note(doc, label, text, color):
    paragraph = doc.add_paragraph(style="Normal")
    paragraph.paragraph_format.left_indent = Pt(8)
    paragraph.paragraph_format.right_indent = Pt(8)
    paragraph.paragraph_format.space_before = Pt(2)
    paragraph.paragraph_format.space_after = Pt(2)
    add_run(paragraph, f"[{label}] ", color=color, bold=True, size=9)
    add_run(paragraph, text, color=color, size=9)
    return paragraph


def add_heading(doc, text, level=1):
    style = {1: "Heading 7", 2: "Heading 8", 3: "Heading 3"}.get(level, "Heading 8")
    paragraph = doc.add_paragraph(style=style if style in doc.styles else "Normal")
    paragraph.paragraph_format.keep_with_next = True
    paragraph.paragraph_format.space_before = Pt(4 if level == 1 else 2)
    paragraph.paragraph_format.space_after = Pt(1)
    if not paragraph.text:
        add_run(paragraph, text, bold=True, size=11 if level == 1 else 10, cn="黑体")
    else:
        paragraph.text = text
    return paragraph


def add_equation(doc, equation, number):
    table = doc.add_table(rows=1, cols=2)
    table.alignment = WD_TABLE_ALIGNMENT.CENTER
    table.autofit = False
    table.columns[0].width = Cm(7.0)
    table.columns[1].width = Cm(0.9)
    left = table.cell(0, 0).paragraphs[0]
    left.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = left.add_run(equation)
    set_run_font(run, cn="Cambria Math", en="Cambria Math", size=10, italic=True)
    right = table.cell(0, 1).paragraphs[0]
    right.alignment = WD_ALIGN_PARAGRAPH.RIGHT
    add_run(right, f"({number})", size=10)
    remove_table_borders(table)
    return table


def remove_table_borders(table):
    tbl_pr = table._tbl.tblPr
    borders = tbl_pr.first_child_found_in("w:tblBorders")
    if borders is None:
        borders = OxmlElement("w:tblBorders")
        tbl_pr.append(borders)
    for edge in ("top", "left", "bottom", "right", "insideH", "insideV"):
        tag = "w:" + edge
        element = borders.find(qn(tag))
        if element is None:
            element = OxmlElement(tag)
            borders.append(element)
        element.set(qn("w:val"), "nil")


def set_cell_border(cell, **kwargs):
    tc_pr = cell._tc.get_or_add_tcPr()
    tc_borders = tc_pr.first_child_found_in("w:tcBorders")
    if tc_borders is None:
        tc_borders = OxmlElement("w:tcBorders")
        tc_pr.append(tc_borders)
    for edge in ("top", "bottom", "left", "right", "insideH", "insideV"):
        if edge not in kwargs:
            continue
        edge_data = kwargs[edge]
        tag = "w:" + edge
        element = tc_borders.find(qn(tag))
        if element is None:
            element = OxmlElement(tag)
            tc_borders.append(element)
        for key, value in edge_data.items():
            element.set(qn("w:" + key), str(value))


def add_table(doc, headers, rows, widths=None, font_size=8, caption=None, english_caption=None):
    if caption:
        p = doc.add_paragraph(style="Normal")
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        add_run(p, caption, bold=True, size=9)
    if english_caption:
        p = doc.add_paragraph(style="Normal")
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        add_run(p, english_caption, size=8)
    table = doc.add_table(rows=1, cols=len(headers))
    table.alignment = WD_TABLE_ALIGNMENT.CENTER
    table.autofit = True
    if widths:
        for index, width in enumerate(widths):
            table.columns[index].width = Cm(width)
    for index, header in enumerate(headers):
        cell = table.rows[0].cells[index]
        cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
        p = cell.paragraphs[0]
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        add_run(p, str(header), bold=True, size=font_size)
    for row in rows:
        cells = table.add_row().cells
        for index, value in enumerate(row):
            color = BLACK
            text = value
            if isinstance(value, tuple):
                text, color = value
            cells[index].vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
            p = cells[index].paragraphs[0]
            p.alignment = WD_ALIGN_PARAGRAPH.CENTER if index > 0 else WD_ALIGN_PARAGRAPH.LEFT
            add_run(p, str(text), color=color, size=font_size)
    remove_table_borders(table)
    border = {"val": "single", "sz": "8", "color": "000000"}
    for cell in table.rows[0].cells:
        set_cell_border(cell, top=border, bottom=border)
    for cell in table.rows[-1].cells:
        set_cell_border(cell, bottom=border)
    return table


def add_picture(doc, path, width_inches, caption, english_caption, note=None):
    paragraph = doc.add_paragraph(style="Normal")
    paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
    paragraph.paragraph_format.keep_with_next = True
    run = paragraph.add_run()
    run.add_picture(str(path), width=Inches(width_inches))
    p = doc.add_paragraph(style="Normal")
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.keep_with_next = True
    add_run(p, caption, bold=True, size=9)
    p = doc.add_paragraph(style="Normal")
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    add_run(p, english_caption, size=8)
    if note:
        add_note(doc, "图件说明", note, PURPLE)


def start_columns(doc, count, start_type=WD_SECTION.CONTINUOUS):
    section = doc.add_section(start_type)
    configure_section(section, count)
    return section


def add_page_break(doc):
    paragraph = doc.add_paragraph()
    paragraph.add_run().add_break(WD_BREAK.PAGE)


def add_title_block(doc):
    p = doc.add_paragraph(style="Subtitle" if "Subtitle" in doc.styles else "Normal")
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    add_run(p, "长走廊退化环境下激光雷达-里程计各向异性融合方法", bold=True, size=17, cn="黑体")
    p = doc.add_paragraph(style="Subtitle" if "Subtitle" in doc.styles else "Normal")
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    add_run(p, "[作者姓名1]1，[作者姓名2]1+", color=RED, size=11)
    p = doc.add_paragraph(style="单位" if "单位" in doc.styles else "Normal")
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    add_run(p, "1. [学校全称] [学院全称]，[省] [市] [邮编]", color=RED, size=9)
    p = doc.add_paragraph(style="单位" if "单位" in doc.styles else "Normal")
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    add_run(p, "+ 通信作者 E-mail：[待填]", color=RED, size=9)


def add_abstracts(doc):
    chinese = (
        "针对无惯性测量单元条件下，二维激光同步定位与建图系统在长直走廊中存在纵向几何约束退化，且在开阔连接区易与轮式里程计产生相互冲突的问题，提出一种方向退化感知与一致性可靠性保护相结合的激光雷达-里程计各向异性融合方法。"
        "首先根据当前二维点云协方差矩阵的特征值和主特征向量估计退化置信度与退化主方向，再通过完整的二维平方根信息矩阵将 Cartographer 前端平移先验从标量约束扩展为各向异性约束。"
        "然后利用 LiDAR local SLAM 与轮式里程计的横向和航向相对运动残差构造一致性异常指标，结合扫描点数、角度覆盖和方向退化置信度判断 LiDAR 可靠性，并通过双阈值迟滞和一阶低通滤波实现异常快速降权与平滑恢复。"
        "现有长走廊数据实验中，方向退化模块触发 74 次，退化方向权重倍率为 2.00～3.55，非退化方向保持为 1.0；相比静态高权重，改进方法避免了将里程计先验同时强加到所有方向。"
    )
    p = doc.add_paragraph(style="Normal (Web)" if "Normal (Web)" in doc.styles else "Normal")
    p.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
    add_run(p, "摘 要：", bold=True, size=10)
    add_run(p, chinese, size=10)
    add_run(p, "[待补：新正式数据上原版与完整方法的 RPE/检查点误差降幅、一致性异常 F1 和 P95 耗时。]", color=RED, bold=True, size=9)

    p = doc.add_paragraph(style="关键词" if "关键词" in doc.styles else "Normal")
    add_run(p, "关键词：", bold=True)
    add_run(p, "激光同步定位与建图；Cartographer 2D；方向性退化；各向异性融合；轮式里程计；一致性异常")
    p = doc.add_paragraph(style="分类号" if "分类号" in doc.styles else "Normal")
    add_run(p, "文献标志码：A    中图分类号：", bold=True)
    add_run(p, "[待编辑/导师确认，候选 TP242]", color=RED, bold=True)

    p = doc.add_paragraph(style="Name" if "Name" in doc.styles else "Normal")
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    add_run(p, "Anisotropic LiDAR-Odometry Fusion for Long-Corridor Degenerate Environments", bold=True, size=14)
    p = doc.add_paragraph(style="Name" if "Name" in doc.styles else "Normal")
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    add_run(p, "[AUTHOR One]1, [AUTHOR Two]1+", color=RED, size=10)
    p = doc.add_paragraph(style="Depart.Correspond" if "Depart.Correspond" in doc.styles else "Normal")
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    add_run(p, "1. [School/Department, University, City Postcode, China]", color=RED, size=9)
    english = (
        "To address longitudinal geometric degeneracy in long corridors and observation conflicts between two-dimensional LiDAR local SLAM and wheel odometry in open connecting areas, an anisotropic fusion method combining directional degeneracy perception with consistency-aware reliability protection is presented. "
        "The eigenvalues and principal eigenvector of the current two-dimensional point-cloud covariance are used to estimate a smoothed degeneracy confidence and its dominant direction. A full two-dimensional square-root information matrix then replaces the scalar translational prior in the Cartographer front-end. "
        "Lateral and yaw residuals between LiDAR local SLAM and wheel-odometry relative motions are further combined with scan population, angular coverage and directional degeneracy confidence to gate a hysteretic dynamic weighting strategy. "
        "In the available corridor sequence, the directional module was activated 74 times; the prior scale along the degenerate direction varied from 2.00 to 3.55 while the non-degenerate direction remained at 1.0. The current evidence supports directional weight injection and reduced side effects compared with globally high static weighting."
    )
    p = doc.add_paragraph(style="Normal (Web)" if "Normal (Web)" in doc.styles else "Normal")
    p.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
    add_run(p, "Abstract: ", bold=True)
    add_run(p, english)
    add_run(p, " [AUTHOR_INPUT_NEEDED: final RPE/checkpoint-error reduction, anomaly-detection F1, and P95 runtime on the frozen test set.]", color=RED, bold=True, size=9)
    p = doc.add_paragraph(style="Key words" if "Key words" in doc.styles else "Normal")
    add_run(p, "Key words: ", bold=True)
    add_run(p, "simultaneous localization and mapping; Cartographer 2D; directional degeneracy; anisotropic fusion; wheel odometry; consistency anomaly")


def add_author_working_pages(doc):
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    add_run(p, "《计算机工程与应用》投稿初稿（作者批注版）", color=BLUE, bold=True, size=18, cn="黑体")
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    add_run(p, "官方写作模板版本：2025-11-03    初稿日期：2026-07-15", color=GRAY, size=10)
    add_note(doc, "使用说明", "本页至“投稿正文开始”之前均为作者工作页，定稿时整页删除。正文中的彩色内容也必须在投稿前处理完。", RED)

    add_heading(doc, "彩色标记约定", 1)
    add_table(
        doc,
        ["颜色", "含义", "投稿前操作"],
        [
            (("黑色", BLACK), "可继续打磨的论文正文", "语言复核"),
            (("绿色", GREEN), "已有源码或实验记录支撑的事实", "与最终表格交叉核对后转黑"),
            (("红色", RED), "作者信息、数据或引用缺口", "必须填写或删除"),
            (("蓝色", BLUE), "写作、实验或排版建议", "执行后删除"),
            (("紫色", PURPLE), "审稿风险和证据边界", "补证据或收缩结论后删除"),
        ],
        font_size=9,
    )

    add_heading(doc, "官方格式要点摘录", 1)
    add_body(doc, "题名通常不超过 25 字；中文摘要约 300 字，不使用“本文/我们”作为主语；关键词 3～8 个；一般论文建议 7500 字以上；正文双栏、10 磅、单倍行距；参考文献建议 20 篇以上。", color=GREEN, indent=False)
    add_body(doc, "绘制图应提供可编辑矢量文件，图像类图件不低于 300 dpi；表格必须为可编辑三线表；公式、变量和矩阵在定稿时需使用公式编辑器统一排版。", color=GREEN, indent=False)

    add_heading(doc, "核心论证句", 1)
    add_body(doc, "在无 IMU 的长走廊二维 SLAM 中，方向性点云几何信息用于判断“哪个方向约束不足”，LiDAR-odometry 一致性与扫描质量信息用于判断“观测冲突时应降低哪一类约束”，两者结合以减少固定权重在走廊与开阔区切换中的系统性失效。", color=BLUE, bold=True, indent=False)

    add_heading(doc, "术语表", 1)
    add_table(
        doc,
        ["统一术语", "定义/缩写", "禁止或限定表述"],
        [
            ("多线三维激光雷达二维投影", "/velodyne_points → /scan", "不称为“三维 SLAM”"),
            ("Cartographer 2D", "位姿 (x,y,θ) 与二维占据栅格", "不涉及 IMU 或六自由度建图"),
            ("方向性退化置信度", "Dconf ∈ [0,1]", "不等同于 Hessian 严格可观性"),
            ("各向异性平移先验", "S=R diag(wlong,wlat) RT", "不声称直接改造了 occupied-space LiDAR 残差"),
            ("一致性异常", "LiDAR local SLAM 与 odometry 相对运动差异", "不等同于已测得真实轮胎打滑"),
            ("LiDAR 扫描质量", "QL：点数+角度覆盖+前端指标", "不解释为绝对位姿协方差"),
        ],
        font_size=8,
    )

    add_heading(doc, "论点-证据映射", 1)
    add_table(
        doc,
        ["主要论点", "当前证据", "状态"],
        [
            ("创新点一真正改变 Ceres 优化权重", "74 次触发，wlong 倍率 2.00～3.55，wlat=1.0", ("已支持", GREEN)),
            ("各向异性方法比静态高权重副作用小", "地图面积、PCA 横向宽度和连通分量对比", ("阶段性支持", GREEN)),
            ("创新点二减少正常段误触发", "Reference 复验 0 触发，weight scale=1.0", ("阶段性支持", GREEN)),
            ("创新点二显著改善轨迹精度", "缺少可靠真值、RPE/F1 和多次重复", ("必须补实验", RED)),
            ("方法适用于不同长走廊", "现有主证据集中于单一场景", ("需多序列泛化", RED)),
        ],
        font_size=8,
    )

    add_heading(doc, "投稿前 P0 必做清单", 1)
    for item in [
        "录制至少 3 条独立的正常完整长走廊往返 bag，去程和回程在电梯厅使用相同路线。",
        "获取真值或至少可测量检查点，计算 RPE；若无连续真值，不伪称计算了 ATE。",
        "完成原版、静态低/高权重、I1 only、I2 only 和 Full Proposed 六组公平对比。",
        "使用独立 ROS 节点注入连续的横向/yaw odometry 增量偏差，同时输出权威标签时间段。",
        "报告 Precision、Recall、F1、检测延迟、恢复时间、均值±标准差和 P95 耗时。",
        "逐篇核验不少于 20 篇参考文献，包括近 3 年本刊/同领域中文文献。",
    ]:
        add_body(doc, "• " + item, color=RED, indent=False)

    add_heading(doc, "源码实现追踪（投稿时删除）", 1)
    add_table(
        doc,
        ["功能", "主要文件", "论文中对应位置"],
        [
            ("方向退化计算", "ceres_scan_matcher_2d.cc/.h", "3.1～3.3"),
            ("2×2 平移残差", "translation_delta_cost_functor_2d.h", "3.4"),
            ("退化参数", "ceres_scan_matcher_options_2d.proto + Lua", "3.5"),
            ("匹配得分传递", "local_trajectory_builder_2d.cc", "3.1/4.2"),
            ("一致性检测", "slip_detector.h", "4.1～4.3"),
            ("后端逐边动态权重", "optimization_problem_2d.cc", "4.4"),
            ("NodeId 时间绑定", "optimization_problem_2d.cc/.h", "4.5"),
            ("ROS 诊断话题", "cartographer_ros/node.cc/.h", "5.2"),
            ("扫描质量门控", "my_navigation/src/scan_quality_gate.cpp", "4.2"),
            ("迟滞/恢复测试", "slip_detector_test.cc", "5.2"),
        ],
        font_size=7,
    )
    add_note(doc, "证据边界", "当前创新点二已实现扫描质量门控与后端 odometry 标量动态降权，但没有将后端 LiDAR 因子改造为新的自定义 CostFunction。正文已按实际工程边界表述。", PURPLE)


def add_manuscript_body(doc):
    add_note(doc, "排版提示", "依官方模板，引言不设标题且不编号；本条投稿时删除。", BLUE)
    intro_paragraphs = [
        "激光同步定位与建图（simultaneous localization and mapping, SLAM）是移动机器人实现自主导航的基础。对平面运动的室内机器人，二维激光 SLAM 能够以较低的计算和存储开销生成占据栅格地图。Cartographer 通过局部扫描匹配、子图构建和后端位姿图优化实现实时二维建图[1]。",
        "然而，室内环境的几何可观性并不恒定。在长直走廊中，两侧平行墙面对横向位移和航向提供较强约束，但沿走廊纵向的几何形状变化较小，扫描匹配可能沿走廊方向滑移。当机器人从走廊进入电梯厅等开阔连接区时，还可能出现激光回波稀疏、角度覆盖减少与航向匹配异常。",
        "轮式里程计可以提供连续的短时运动预测，但其精度受轮径标定、左右轮速比例、地面摩擦和轮胎滑动影响。将 odometry 权重设置过低，难以在走廊纵向退化时维持轨迹尺度；将其全局设置过高，则会把累积航向偏置或一致性异常强制传递给位姿图。因此，固定标量权重无法表达“方向性退化”和“时变可靠性”这两类不同问题。",
        "现有二维激光 SLAM 研究已形成粒子滤波、扫描匹配和图优化等主要路线[2-5]。相关退化处理方法主要利用匹配得分、优化 Hessian、特征值或多传感器残差评估当前约束质量。但在无 IMU 的工程平台上，仍需要一种计算量较低、能够区分走廊纵向与横向，并能处理 LiDAR-odometry 观测冲突的可配置方法。",
        "为此，设计一种面向 Cartographer 2D 的方向自适应融合方法。第一，使用当前点云二维协方差的条件数和主特征向量作为方向性退化的低开销代理指标，将 Ceres 前端平移先验由标量扩展为 2×2 各向异性平方根信息矩阵。第二，将 LiDAR-odometry 一致性检测与 LiDAR 质量判别分开，在 LiDAR 可靠时对异常 odometry 边标量降权，在 LiDAR 稀疏时优先通过输入质量门控保护前端。第三，通过节点时间绑定解决 Cartographer 前后端异步条件下的历史边指标对齐问题。",
    ]
    for paragraph in intro_paragraphs:
        add_body(doc, paragraph)
    add_note(doc, "待补引用", "引言中 [2-5] 及退化/多传感器相关论述需与最终参考文献逐一对齐，不得用候选文献替代实际阅读。", RED)

    add_heading(doc, "1 系统模型与问题定义", 1)
    add_heading(doc, "1.1 传感器配置与二维投影", 2)
    add_body(doc, "机器人搭载多线三维激光雷达和轮式底盘，不使用 IMU。原始点云话题为 /velodyne_points，轮式里程计话题为 /odom。对经外参变换后的三维点 pi=(xi,yi,zi)T，依次进行高度带与量程筛选，并将水平角度相同的点保留最小量程，得到标准 LaserScan。")
    add_equation(doc, "zmin ≤ zi ≤ zmax,    rmin ≤ √(xi²+yi²) ≤ rmax", 1)
    add_body(doc, "采用三维雷达硬件但运行二维 SLAM 不存在矛盾：多线回波便于离线重新选择高度与量程，而机器人运动状态和导航地图仍是平面的。由于本系统没有 IMU，直接使用 Cartographer 3D 会引入缺少重力与姿态约束的六自由度估计，与本研究目标不符。")

    start_columns(doc, 1)
    add_picture(
        doc,
        DOCS / "paper_assets" / "system_pipeline.png",
        7.0,
        "图1 方向退化感知与一致性可靠性保护总体框架",
        "Fig.1 Overall framework of directional degeneracy perception and consistency-aware reliability protection",
        "已输出可编辑 SVG/PDF 与 600 dpi TIFF。定稿时检查英文图题、变量斜体和线宽是否完全符合官方模板。",
    )
    start_columns(doc, 2)

    add_heading(doc, "1.2 Cartographer 2D 优化模型", 2)
    add_body(doc, "Cartographer 2D 前端以位姿预测为初值，通过占据空间代价、平移先验和旋转先验对当前扫描与局部子图进行 Ceres 优化。简化目标函数为：")
    add_equation(doc, "ξ* = arg minξ [ Eocc(ξ) + Etrans(ξ) + Erot(ξ) ]", 2)
    add_body(doc, "其中，Eocc 为激光点与占据栅格的匹配代价，Etrans 和 Erot 分别为相对初始预测位姿的平移和旋转先验。原始平移残差 r=w(t-t0) 对 x、y 方向采用相同权重，不能表达长走廊的方向性退化。")
    add_body(doc, "后端位姿图中，相邻轨迹节点之间的 odometry 残差使用 Lua 配置的平移和旋转权重。若该权重始终保持较高，则轮速航向累积偏置会持续传入地图；若始终保持较低，则长走廊纵向缺少足够运动支撑。")

    add_heading(doc, "2 方向退化感知的各向异性融合", 1)
    add_heading(doc, "2.1 点云协方差与退化主方向", 2)
    add_body(doc, "设当前滤波后二维点云为 P={pi}i=1N，pi=[xi,yi]T。点云均值和协方差矩阵分别为：")
    add_equation(doc, "p̄ = (1/N) Σ pi", 3)
    add_equation(doc, "Σp = (1/N) Σ (pi-p̄)(pi-p̄)T", 4)
    add_body(doc, "对对称矩阵 Σp 使用 SelfAdjointEigenSolver 进行特征分解。令 λmax≥λmin，最大特征值对应的单位特征向量 vlong 作为点云主延伸方向，与其正交的 vlat 作为横向。为避免除零和少量点造成的数值不稳定，对 λmin 设置正数下界 ε，并对有效点数设置最小要求。")
    add_equation(doc, "κ = λmax / max(λmin, ε)", 5)
    add_note(doc, "证据边界", "κ 表辁点云整体的二阶几何形状，是当前工程版的退化代理指标，不等价于 scan matcher Hessian 的严格可观性。最终实验应补充 κ 与纵向 RPE/匹配波动的相关性图。", PURPLE)

    add_heading(doc, "2.2 退化置信度与时间平滑", 2)
    add_body(doc, "为避免条件数在阈值附近导致权重硬切换，使用 Sigmoid 软映射将 κ 转换为 [0,1] 范围的原始退化置信度：")
    add_equation(doc, "Draw = 1 / {1 + exp[-s(κ-κ0)]}", 6)
    add_body(doc, "式中，κ0 是条件数软阈值中心，s 为映射斜率。随后对置信度使用指数滑动平均：")
    add_equation(doc, "Dk = αD Draw,k + (1-αD)Dk-1", 7)
    add_body(doc, "特征向量 v 与 -v 表示同一方向轴。在方向滤波前，若当前方向与上一帧方向的点积为负，则反转当前特征向量，以避免 180° 方向跳变。")

    add_heading(doc, "2.3 二维各向异性平移先验", 2)
    add_body(doc, "方向权重不再通过 Dx、Dy 的标量投影重复拆分，而是直接在特征基中设置纵向与横向权重，再旋转回局部雷达坐标系：")
    add_equation(doc, "R = [vlong, vlat]", 8)
    add_equation(doc, "S = R diag(wlong, wlat) RT", 9)
    add_body(doc, "对平移误差 e=t-t0，Ceres 平移先验残差从 r=w0e 修改为 r=Se。其中 S 为平方根信息矩阵。点云退化时，通过增大 wlong 强化运动先验，而 wlat 可保持接近基础权重，从而保留 LiDAR 对两侧墙面的横向校正作用。")
    add_equation(doc, "wlong = w0(1+αlongD) / max(1-βlongD, smin)", 10)
    add_equation(doc, "wlat = w0(1+αlatD) / max(1-βlatD, smin)", 11)
    add_body(doc, "需要强调的是，占据空间代价由每个激光点在栅格上的标量概率插值构成，不是显式的 [dx,dy]T 残差。因此当前方法保留 occupied-space cost 不变，通过各向异性平移先验改变 LiDAR 匹配与 odometry/运动预测之间的方向性相对影响。")
    add_note(doc, "审稿风险", "正文不得将 S 说成“直接修改 LiDAR occupied-space 信息矩阵”。实际改动对象是 TranslationDeltaCostFunctor2D 的平移先验。", PURPLE)

    add_heading(doc, "2.4 Ceres 自动微分兼容实现", 2)
    add_body(doc, "为避免在 Ceres Jet 类型中重复计算特征分解，S 在构建优化问题前以 Eigen::Matrix<double,2,2> 预计算。代价函数中先用模板类型 T 构造误差向量 Eigen::Matrix<T,2,1>，再由 double 矩阵左乘该向量。这种实现保留自动微分链路，且每帧新增计算为 O(N) 协方差累加和常数规模的 2×2 特征分解。")
    add_note(doc, "待补性能", "需在正式实验平台上用统一计时点统计平均、P95 和最大耗时，以验证每帧新增开销 <2 ms 的工程目标。", RED)

    add_heading(doc, "3 一致性异常下的可靠性保护", 1)
    add_heading(doc, "3.1 LiDAR-odometry 相对运动残差", 2)
    add_body(doc, "对相邻后端轨迹节点 i 和 j，定义 LiDAR local SLAM 相对位姿 TLij 和轮式里程计相对位姿 TOij。两种观测的相对差异为：")
    add_equation(doc, "ΔTij = (TOij)-1 TLij", 12)
    add_body(doc, "从 ΔTij 提取横向平移残差 ey 和归一化航向残差 eθ，并构造线性一致性异常代理分数：")
    add_equation(doc, "ey=|Δty|,    eθ=|atan2(ΔR10,ΔR00)|", 13)
    add_equation(doc, "Craw = wy ey + wθ eθ", 14)
    add_body(doc, "二维旋转通过 atan2 直接从旋转矩阵提取航向差，并归一化到 [-π,π]，不涉及三维欧拉角分解和万向节死锁。长走廊纵向 LiDAR 本身可能退化，因此当前版本不把纵向差异作为 odometry 异常的主要判据。")
    add_note(doc, "术语边界", "一致性差异只能证明两个相对运动不一致，不能在没有外部真值时直接证明真实轮胎打滑。论文统一使用“一致性异常”。", PURPLE)

    add_heading(doc, "3.2 LiDAR 扫描质量与故障归因", 2)
    add_body(doc, "仅使用 Craw 无法判断哪个传感器出错。为此，对投影后 LaserScan 统计有效量程数 Nvalid，将扫描角度范围划分为 M 个分区，并统计满足最少点数的活跃分区数 Nactive。当 Nvalid 或 Nactive 低于阈值时，该帧不进入 local scan matcher，避免少量单侧回波驱动错误航向匹配。")
    add_body(doc, "当扫描质量满足要求且创新点一表明横向/航向结构仍可用时，允许一致性差异更新 odometry 异常状态。当 LiDAR 可靠性不可用或低于门限时，保持上一状态和权重，不使用不可靠 LiDAR 观测反向处罚 odometry。")

    add_heading(doc, "3.3 双阈值迟滞与动态权重", 2)
    add_body(doc, "对通过 LiDAR 可靠性门控的一致性分数 C，设置异常触发高阈值 Chigh 和正常恢复低阈值 Clow，且 Clow<Chigh。C>Chigh 时进入异常状态，C<Clow 时进入正常状态，中间区间保持上一状态。")
    add_body(doc, "异常状态下将 odometry 权重缩放比例 γ 快速降为 γmin；恢复正常后，使用一阶低通滤波逐步恢复到 γmax：")
    add_equation(doc, "γk = γmin,                         anomaly", 15)
    add_equation(doc, "γk = αrγmax + (1-αr)γk-1,    normal", 16)
    add_body(doc, "后端每条 odometry 边的实际平移与旋转权重分别为 wt,dyn=γwt,base 和 wθ,dyn=γwθ,base。当前实现保留标量缩放，不修改 Ceres 后端 CostFunction 结构，以降低工程风险并保留原始约束形式。")

    add_heading(doc, "3.4 前后端时间对齐", 2)
    add_body(doc, "Cartographer local SLAM 和 pose graph 优化在不同工作线程中执行。如果将前端“最新一帧”指标直接应用到后端所有历史边，会产生明显时间错配。实现中为每个前端退化指标保存传感器时间，后端轨迹节点创建时查找时间最近的指标并绑定至 NodeId。后续每次 Solve() 处理历史边时，直接读取节点绑定值，不再依赖全局 latest metric。")
    add_body(doc, "前端指标缓存的写入和查询使用互斥锁保护，锁内只执行容器操作和小对象拷贝，不执行 ROS 发布或 Ceres 求解。节点被 trim 时同步删除绑定数据，避免无界缓存。")
    add_note(doc, "篇幅建议", "本小节在最终稿中控制为 1 段。它证明方法可复现且无数据竞争，但不应与核心算法竞争篇幅。", BLUE)

    add_heading(doc, "4 实验与结果", 1)
    add_heading(doc, "4.1 实验平台与数据", 2)
    add_body(doc, "实验平台为轮式移动底盘、多线三维激光雷达和 ROS 工控机。雷达原始点云以约 10 Hz 记录，轮式里程计以约 50 Hz 记录。所有对比使用相同 bag、投影参数、静态外参、初始位姿、关键帧策略和回环参数，仅修改对应消融项的开关与权重方式。")
    add_body(doc, "受实验场地条件限制，正式重复性实验将在同一条物理长走廊独立采集 3 次完整往返序列。每条序列都对应一次新的真实机器人行驶与单独 rosbag，而不是对同一 bag 重复播放。第 1 条序列用于参数验证，第 2、3 条序列在参数冻结后用于测试和重复性评价。三次采集使用相同起点、初始朝向、电梯厅路线、掉头区域和回程路线。")
    add_note(doc, "作者待填", "请填写底盘型号、雷达型号/线数/视场角、CPU、内存、Ubuntu/ROS/Ceres 版本、雷达安装高度和外参标定方法。", RED)
    add_table(
        doc,
        ["数据序列", "用途", "当前状态"],
        [
            ("corridor_01.bag（约 374 s）", "创新点一三组阶段性实验", ("已完成", GREEN)),
            ("corridor_01_slip_injected.bag", "轻度 odometry 异常链路验证", ("阶段性，证据不足", PURPLE)),
            ("corridor_new.bag（约 487 s）", "开阔区 LiDAR 稀疏与 odometry 长时漂移压力测试", ("失败案例，不作主效果图", PURPLE)),
            ("corridor_repeat_01/02/03", "同一长走廊的 3 次独立录制；01 验证，02/03 测试", ("待录制", RED)),
        ],
        caption="表1 数据序列与实验用途",
        english_caption="Table 1 Data sequences and experimental purposes",
        font_size=7,
    )

    add_heading(doc, "4.2 对比方法与消融设置", 2)
    add_table(
        doc,
        ["组别", "创新点1", "创新点2", "作用"],
        [
            ("Original Cartographer", "关闭", "关闭", "原版基线"),
            ("Static Low", "关闭", "关闭", "静态低 odometry/平移先验"),
            ("Static High", "关闭", "关闭", "静态高 odometry/平移先验"),
            ("I1 Only", "开启", "关闭", "验证方向各向异性"),
            ("I2 Only", "关闭", "开启", "验证一致性保护"),
            ("Full Proposed", "开启", "开启", "验证模块组合"),
        ],
        caption="表2 六组对比与消融实验",
        english_caption="Table 2 Comparison and ablation configurations",
        font_size=7,
    )
    add_note(doc, "公平性要求", "Original Cartographer 需要在验证集上合理调参，不能故意使用明显较差参数。所有正式结果应记录 Git commit、Lua 文件、bag SHA256 和运行命令。", BLUE)

    add_heading(doc, "4.3 评价指标与统计规则", 2)
    add_body(doc, "若可获得连续二维轨迹真值，在不进行尺度修正的前提下，通过刚体 SE(2) 对齐计算绝对轨迹误差（absolute trajectory error, ATE）与相对位姿误差（relative pose error, RPE）。若只有实测检查点，则报告检查点位置误差和终点闭合误差，不使用 ATE 名称。")
    add_body(doc, "地图质量采用走廊实测宽度误差、墙体 RANSAC 直线拟合 RMS、去程/回程墙体重影距离和占用栅格连通分量作为辅助指标。地图面积和 PCA 横向宽度只用于阶段性诊断，不替代标准轨迹误差。")
    add_body(doc, "一致性异常注入节点应输出独立标签文件，根据已知注入时段统计 Precision、Recall、F1、首次触发延迟和恢复时间。统计中的独立样本 n 是独立录制的 bag 或独立异常事件，不是高频 ROS 话题样本点数。")
    add_note(doc, "统计 P0", "正式稿需明确 n、重复次数、均值±标准差、异常事件标注规则和异常检测阈值的验证集/测试集分离。不得将同一 bag 的数万个定时发布样本当成数万个独立样本。", RED)

    add_heading(doc, "4.4 方向性退化的阶段性结果", 2)
    add_body(doc, "在 corridor_01.bag 上运行静态低权重、静态高权重和动态各向异性三组实验。动态方法在该数据中触发 74 次，退化主方向权重缩放倍率为 2.00359～3.54978，平均为 2.46444，非退化方向保持 1.0。这一运行结果表明方向性权重已实际进入 Ceres 平移先验残差，而不是仅生成诊断日志。", color=GREEN)
    add_table(
        doc,
        ["指标", "Static Low", "Static High", "Proposed I1"],
        [
            ("地图面积/m²", "845.65", "1736.34", "932.56"),
            ("PCA 主方向 P95/m", "49.50", "48.37", "49.20"),
            ("PCA 横向宽度 P90/m", "2.139", "2.224", "2.129"),
            ("最大连通分量占比", "0.1988", "0.1953", "0.2035"),
        ],
        caption="表3 创新点一现有三组地图诊断指标",
        english_caption="Table 3 Existing diagnostic map metrics for Innovation 1",
        font_size=7,
    )
    add_body(doc, "静态高权重组的地图面积和占用连通分量明显增加，表明全方向强运动先验会将 odometry 偏差同时引入非退化方向。Proposed 的地图面积和主方向长度更接近静态低权重，PCA 横向宽度 P90 为三组中最小。这些证据支持“避免静态高权重的全方向副作用”，但不足以声称 Proposed 在所有指标上绝对优于 Static Low。", color=GREEN)

    start_columns(doc, 1)
    add_picture(
        doc,
        ROOT / "paper_exp" / "innovation1_directional_adaptive_fusion" / "map_comparison.png",
        7.0,
        "图2 创新点一三组地图对比（阶段性结果）",
        "Fig.2 Three-way map comparison for Innovation 1 (preliminary result)",
        "该图可用于内部初稿，正式投稿前需用统一坐标轴、统一比例尺和中英文子图标签重绘为矢量图。",
    )
    start_columns(doc, 2)

    add_heading(doc, "4.5 一致性异常的阶段性结果", 2)
    add_body(doc, "现有轻度 odometry 异常注入实验表明，Reference 原始数据上一致性异常触发数为 0，最小权重缩放为 1.0，说明 LiDAR 可靠性门控能够抑制当前参数下的正常段误触发。在轻度注入数据上，Proposed 记录到 1 次短时后端触发，权重缩放最低为 0.145；相比静态高 odometry 权重，地图面积从 1134.75 m² 降至 1120.56 m²，占据连通分量从 143 降至 128。", color=GREEN)
    add_body(doc, "但是，Proposed 的 PCA 横向宽度 P90 从静态高权重的 2.722 m 变为 2.741 m，未取得改善；相比静态低权重，其地图指标也不是全面最优。因此，当前结果只能支持“系统完成了可靠性门控与短时动态降权，并对静态高权重的地图膨胀/碎裂有限缓解趋势”，不能声称已显著降低轨迹误差。", color=PURPLE)

    start_columns(doc, 1)
    add_picture(
        doc,
        DOCS / "paper_assets" / "innovation2_preliminary_timeline.png",
        7.0,
        "图3 创新点二现有运行记录的时间对齐诊断",
        "Fig.3 Time-aligned diagnostic traces from the current Innovation 2 run",
        "当前日志没有保存权威注入起止时间，所以图中没有伪造红色注入阴影。最终实验必须由注入节点同步输出标签。",
    )
    start_columns(doc, 2)

    add_heading(doc, "4.6 开阔区压力测试", 2)
    add_body(doc, "corridor_new.bag 在约 90.0～104.5 s 内出现明显扫描稀疏，95.2～98.3 s 最严重时，原始点数约为 90～509，有效二维角度格最低约为 22。该时段之后 LiDAR 前端航向出现接近 90° 的异常；odometry 没有单帧跳变，但在去程直线段长时累积约 33.3° 假航向。这表明该数同时包含“短时 LiDAR 失配”和“长时 odometry 漂移”两个相反故障源。", color=GREEN)
    add_body(doc, "在前 130 s 诊断中，低 odometry 权重加扫描质量门控的轨迹范围约为 45.2 m×7.35 m，未门控的低权重轨迹约为 37 m×17.1 m，说明质量门控对该短时航向失配有抑制作用。但整包回程地图仍受 odometry 长时漂移影响而产生重影，因此该 bag 只作为故障归因和压力测试，不作为论文最终主效果图。", color=PURPLE)

    start_columns(doc, 1)
    add_heading(doc, "4.7 最终结果占位表", 2)
    add_table(
        doc,
        ["方法", "RPE平移↓", "RPE航向↓", "墙体RMS↓", "闭合误差↓"],
        [
            ("Original", ("[待补]", RED), ("[待补]", RED), ("[待补]", RED), ("[待补]", RED)),
            ("Static Low", ("[待补]", RED), ("[待补]", RED), ("[待补]", RED), ("[待补]", RED)),
            ("Static High", ("[待补]", RED), ("[待补]", RED), ("[待补]", RED), ("[待补]", RED)),
            ("I1 Only", ("[待补]", RED), ("[待补]", RED), ("[待补]", RED), ("[待补]", RED)),
            ("I2 Only", ("[待补]", RED), ("[待补]", RED), ("[待补]", RED), ("[待补]", RED)),
            ("Full Proposed", ("[待补]", RED), ("[待补]", RED), ("[待补]", RED), ("[待补]", RED)),
        ],
        caption="表4 正式测试集轨迹与地图质量结果占位",
        english_caption="Table 4 Placeholder for final trajectory and map-quality results",
        font_size=7,
    )
    add_table(
        doc,
        ["方法", "Precision↑", "Recall↑", "F1↑", "延迟/s↓", "P95耗时/ms↓"],
        [
            ("I2 Only", ("[待补]", RED), ("[待补]", RED), ("[待补]", RED), ("[待补]", RED), ("[待补]", RED)),
            ("Full Proposed", ("[待补]", RED), ("[待补]", RED), ("[待补]", RED), ("[待补]", RED), ("[待补]", RED)),
        ],
        caption="表5 一致性异常检测与计算开销结果占位",
        english_caption="Table 5 Placeholder for anomaly-detection and runtime results",
        font_size=7,
    )
    add_note(doc, "表格规则", "最终表格应报告独立 bag/异常事件上的均值±标准差，并在表头标注指标趋势和单位。不要为了表格完整填入估计值。", BLUE)
    start_columns(doc, 2, WD_SECTION.NEW_PAGE)

    add_heading(doc, "5 讨论", 1)
    add_heading(doc, "5.1 方向性调权的作用", 2)
    add_body(doc, "现有实验表明，各向异性方法的核心优势不是将 odometry 全局提高，而是根据点云主方向对退化方向和非退化方向分别调度。这一结构使系统可以在长走廊纵向借助运动预测，同时保留 LiDAR 在横向和航向上的几何校正。相比静态高权重，阶段性结果支持该方法能够避免明显的全方向地图膨胀。")
    add_heading(doc, "5.2 一致性保护的作用边界", 2)
    add_body(doc, "一致性异常检测本身不能判断 LiDAR 或 odometry 中谁是错误源。开阔区压力数据展示了一个典型反例：短时 LiDAR 稀疏时应避免使用错误激光相对位姿处罚 odometry，而 LiDAR 结构恢复后与 odometry 持续冲突时，又不应保持全局高 odometry 权重。因此，质量判别是一致性调权前的必要环节，而不是一个与创新点二无关的附加功能。")
    add_body(doc, "当前创新点二的实验增益较小，且缺少外部真值。因此它在小论文中定位为“方向自适应融合的可靠性保护”，而不是独立宣称已完成通用物理打滑检测。只有在完成带权威时间标签的异常注入和 F1/RPE 统计后，才能增强结论。", color=PURPLE)

    add_heading(doc, "5.3 局限性", 2)
    limitations = [
        "多线点云的二维投影丢失高度信息，过宽高度带可能将不同高度物体折叠到同一平面。",
        "点云协方差条件数是几何代理指标，无法替代扫描匹配 Hessian 的严格不确定性建模。",
        "LiDAR local SLAM 的初始位姿预测可使用 odometry，因此两种相对运动不是完全统计独立。",
        "无 IMU 时不能直接观测车体加速度突变，纯纵向轮胎空转在 LiDAR 纵向同时退化时可能不可识别。",
        "LiDAR 和 odometry 同时不可靠时，系统只能进入保守传播，无法从两个错误源中恢复真实运动。",
        "当前参数为长走廊实验配置，其跨场景泛化性尚需在多条独立数据序列上验证。",
    ]
    for item in limitations:
        add_body(doc, "• " + item, indent=False)

    add_heading(doc, "6 结束语", 1)
    add_body(doc, "针对无 IMU 长走廊二维 SLAM 中的方向性几何退化与 LiDAR-odometry 观测冲突问题，提出了一种方向退化感知和一致性可靠性保护相结合的融合方法。该方法通过点云协方差特征分解获取退化主方向，使用平滑置信度和 2×2 平方根信息矩阵构造 Cartographer 前端各向异性平移先验；同时结合扫描质量和相对运动残差，对后端 odometry 约束进行快速降权与平滑恢复。")
    add_body(doc, "现有实验已证实方向性权重实际注入 Ceres 优化，并在保持静态低权重地图紧凑性的同时，减少了静态高权重对非退化方向的副作用。一致性保护已完成正常段误触发抑制和轻度异常的短时动态降权链路验证，但其轨迹精度改善仍需要新数据集上的 RPE、F1 和多次重复实验支撑。", color=GREEN)
    add_note(doc, "定稿替换", "最后一句在补齐正式实验后，应替换为带数值的决定性结论，不在结束语中重复所有表格。", BLUE)


def add_references_and_author_info(doc):
    add_heading(doc, "参考文献", 1)
    references = [
        "[1] HESS W, KOHLER D, RAPP H, et al. Real-time loop closure in 2D LIDAR SLAM[C]//2016 IEEE International Conference on Robotics and Automation. Stockholm: IEEE, 2016: 1271-1278.",
        "[2] GRISETTI G, STACHNISS C, BURGARD W. Improved techniques for grid mapping with Rao-Blackwellized particle filters[J]. IEEE Transactions on Robotics, 2007, 23(1): 34-46.",
        "[3] KOHLBRECHER S, VON STRYK O, MEYER J, et al. A flexible and scalable SLAM system with full 3D motion estimation[C]//2011 IEEE International Symposium on Safety, Security, and Rescue Robotics. Kyoto: IEEE, 2011: 155-160.",
        "[4] OLSON E B. Real-time correlative scan matching[C]//2009 IEEE International Conference on Robotics and Automation. Kobe: IEEE, 2009: 4387-4393.",
        "[5] CENSI A. An ICP variant using a point-to-line metric[C]//2008 IEEE International Conference on Robotics and Automation. Pasadena: IEEE, 2008: 19-25.",
        "[6] KÜMMERLE R, GRISETTI G, STRASDAT H, et al. g2o: A general framework for graph optimization[C]//2011 IEEE International Conference on Robotics and Automation. Shanghai: IEEE, 2011: 3607-3613.",
        "[7] DELLAERT F, KAESS M. Square Root SAM: Simultaneous localization and mapping via square root information smoothing[J]. The International Journal of Robotics Research, 2006, 25(12): 1181-1203.",
        "[8] SEGAL A, HAEHNEL D, THRUN S. Generalized-ICP[C]//Robotics: Science and Systems. Seattle, 2009.",
        "[9] ZHANG J, SINGH S. LOAM: Lidar odometry and mapping in real-time[C]//Robotics: Science and Systems. Berkeley, 2014.",
        "[10] CERES SOLVER TEAM. Ceres Solver[EB/OL]. [2026-07-15]. http://ceres-solver.org/.",
    ]
    for ref in references:
        p = doc.add_paragraph(style="参考文献" if "参考文献" in doc.styles else "Normal")
        p.paragraph_format.left_indent = Pt(14)
        p.paragraph_format.first_line_indent = Pt(-14)
        add_run(p, ref, color=PURPLE, size=8)
    add_note(doc, "文献核验", "上述为基础候选文献，不表示已完成逐篇通读。投稿前必须核对 DOI、页码、题名、正文引用位置，并增补至少 10 篇近三年 LiDAR SLAM/退化/可靠融合文献及相关中文文献。中文文献需使用原文提供的英文题名，不自行翻译。", RED)
    for index, topic in enumerate(
        [
            "近三年二维 LiDAR SLAM 综述/对比",
            "长走廊或几何退化检测",
            "LiDAR 扫描匹配不确定性/可观性",
            "LiDAR-wheel odometry 动态融合",
            "轮式里程计打滑/一致性异常检测",
            "鲁棒图优化或自适应信息矩阵",
            "《计算机工程与应用》近三年相关论文",
            "三维多线点云二维投影建图",
            "SLAM 轨迹 ATE/RPE 评价规范",
            "占据栅格地图质量评价",
        ],
        start=11,
    ):
        p = doc.add_paragraph(style="参考文献" if "参考文献" in doc.styles else "Normal")
        add_run(p, f"[{index}] [待核验文献：{topic}]", color=RED, size=8)

    add_heading(doc, "作者信息（待填）", 1)
    add_body(doc, "基金项目：[基金名称、类别与项目号；若无则删除]", color=RED, indent=False)
    add_body(doc, "第一作者：[姓名]([ 出生年]-)，[性别]，[职称]，[学历]，研究方向为[待填]。", color=RED, indent=False)
    add_body(doc, "通信作者：[姓名]，[职称]，[学历]，E-mail：[待填]。", color=RED, indent=False)
    add_body(doc, "联系人：[待填]；通讯地址与邮编：[待填]；手机：[待填]；长期有效邮箱：[待填]。", color=RED, indent=False)
    add_note(doc, "终稿自审", "删除全部彩色批注后，再检查：题名长度、中英文摘要对应、缩略语首次定义、图表双语题、公式变量格式、文献顺序、作者顺序、基金与联系方式。", BLUE)


def build_document():
    if not TEMPLATE.exists():
        raise FileNotFoundError(f"Official template not found: {TEMPLATE}")
    doc = Document(str(TEMPLATE))
    clear_document_body(doc)
    configure_styles(doc)
    for section in doc.sections:
        configure_section(section, 1)

    props = doc.core_properties
    props.title = "长走廊退化环境下激光雷达-里程计各向异性融合方法"
    props.subject = "《计算机工程与应用》投稿初稿（作者批注版）"
    props.keywords = "Cartographer 2D; directional degeneracy; anisotropic fusion; consistency anomaly"
    props.comments = "Generated from the official 2025-11-03 writing template. Colored text is author-only annotation."

    add_author_working_pages(doc)
    add_page_break(doc)
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    add_run(p, "投稿正文开始", color=BLUE, bold=True, size=12, cn="黑体")
    add_note(doc, "定稿操作", "投稿前删除此标题以及前面所有作者工作页。", BLUE)
    add_title_block(doc)
    add_abstracts(doc)
    start_columns(doc, 2)
    add_manuscript_body(doc)
    add_references_and_author_info(doc)

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    doc.save(str(OUTPUT))
    print(OUTPUT)


if __name__ == "__main__":
    build_document()
