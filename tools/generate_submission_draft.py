#!/usr/bin/env python3
"""Generate the evidence-bounded Chinese manuscript from the journal DOCX template.

This deliberately uses only the Python standard library so the workspace does not
need a Word-generation dependency.  The output is a normal editable .docx: the
journal template supplies styles, headers and page settings; this script replaces
its body and embeds the project-owned system diagram.
"""

from __future__ import annotations

import html
import shutil
import zipfile
from datetime import datetime, timezone
from pathlib import Path
from xml.etree import ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
TEMPLATE = ROOT / "docs" / "计算机工程与应用_官方论文写作模板_20251103.docx"
OUTPUT = ROOT / "docs" / "长走廊退化环境下激光雷达-里程计各向异性融合方法_冻结证据版初稿.docx"
OUTPUT_OPENABLE = ROOT / "docs" / "paper_draft_openable.docx"
PIPELINE = ROOT / "docs" / "paper_assets" / "system_pipeline.png"

W = "http://schemas.openxmlformats.org/wordprocessingml/2006/main"
R = "http://schemas.openxmlformats.org/officeDocument/2006/relationships"


def x(value: str) -> str:
    return html.escape(value, quote=False)


def run(text: str, *, bold: bool = False, italic: bool = False,
        size: int = 20, east_asia: str = "宋体", ascii_font: str = "Times New Roman") -> str:
    props = [f'<w:rFonts w:ascii="{ascii_font}" w:hAnsi="{ascii_font}" w:eastAsia="{east_asia}"/>']
    if bold:
        props.append("<w:b/><w:bCs/>")
    if italic:
        props.append("<w:i/><w:iCs/>")
    props.append(f'<w:sz w:val="{size}"/><w:szCs w:val="{size}"/>')
    preserve = ' xml:space="preserve"' if text[:1].isspace() or text[-1:].isspace() else ""
    return f'<w:r><w:rPr>{"".join(props)}</w:rPr><w:t{preserve}>{x(text)}</w:t></w:r>'


def paragraph(text: str = "", *, align: str = "both", size: int = 20,
              bold: bool = False, italic: bool = False, east_asia: str = "宋体",
              first_line: bool = False, before: int = 0, after: int = 0,
              keep_next: bool = False, keep_lines: bool = False,
              section: str | None = None) -> str:
    ppr = []
    if keep_next:
        ppr.append('<w:keepNext/>')
    if keep_lines:
        ppr.append('<w:keepLines/>')
    if before or after:
        ppr.append(f'<w:spacing w:before="{before}" w:after="{after}" w:line="240" w:lineRule="auto"/>')
    else:
        ppr.append('<w:spacing w:line="240" w:lineRule="auto"/>')
    if first_line:
        ppr.append('<w:ind w:firstLine="400"/>')
    ppr.append(f'<w:jc w:val="{align}"/>')
    if section:
        ppr.append(section)
    body = run(text, bold=bold, italic=italic, size=size, east_asia=east_asia) if text else '<w:r/>'
    return f'<w:p><w:pPr>{"".join(ppr)}</w:pPr>{body}</w:p>'


def rich_paragraph(parts: list[tuple[str, dict]], *, align: str = "both", size: int = 20,
                   first_line: bool = False, before: int = 0, after: int = 0) -> str:
    ppr = []
    if before or after:
        ppr.append(f'<w:spacing w:before="{before}" w:after="{after}" w:line="240" w:lineRule="auto"/>')
    else:
        ppr.append('<w:spacing w:line="240" w:lineRule="auto"/>')
    if first_line:
        ppr.append('<w:ind w:firstLine="400"/>')
    ppr.append(f'<w:jc w:val="{align}"/>')
    runs = "".join(run(text, size=size, **options) for text, options in parts)
    return f'<w:p><w:pPr>{"".join(ppr)}</w:pPr>{runs}</w:p>'


def heading(text: str, level: int) -> str:
    if level == 1:
        return paragraph(text, align="left", size=24, bold=True, east_asia="黑体",
                         before=180, after=80, keep_next=True, keep_lines=True)
    return paragraph(text, align="left", size=21, bold=True, east_asia="黑体",
                     before=120, after=40, keep_next=True, keep_lines=True)


def formula(text: str, number: int) -> str:
    return rich_paragraph(
        [(text, {"italic": True, "east_asia": "Times New Roman"}), (f"    ({number})", {"east_asia": "Times New Roman"})],
        align="center", size=18, before=40, after=40)


def cell(text: str, *, header: bool = False, width: int = 1700) -> str:
    p = paragraph(text, align="center", size=16, bold=header, east_asia="宋体", before=0, after=0)
    shading = '<w:shd w:val="clear" w:fill="E7E6E6"/>' if header else ""
    return f'<w:tc><w:tcPr><w:tcW w:w="{width}" w:type="dxa"/>{shading}<w:vAlign w:val="center"/></w:tcPr>{p}</w:tc>'


def table(headers: list[str], rows: list[list[str]], widths: list[int]) -> str:
    grid = "".join(f'<w:gridCol w:w="{width}"/>' for width in widths)
    head = '<w:tr>' + "".join(cell(text, header=True, width=widths[i]) for i, text in enumerate(headers)) + '</w:tr>'
    body = "".join(
        '<w:tr>' + "".join(cell(text, width=widths[i]) for i, text in enumerate(row)) + '</w:tr>'
        for row in rows
    )
    borders = ('<w:tblBorders><w:top w:val="single" w:sz="8" w:color="000000"/>'
               '<w:left w:val="nil"/><w:bottom w:val="single" w:sz="8" w:color="000000"/>'
               '<w:right w:val="nil"/><w:insideH w:val="single" w:sz="4" w:color="666666"/>'
               '<w:insideV w:val="nil"/></w:tblBorders>')
    return (f'<w:tbl><w:tblPr><w:tblW w:w="0" w:type="auto"/>{borders}'
            '<w:tblLayout w:type="fixed"/></w:tblPr><w:tblGrid>' + grid + '</w:tblGrid>' + head + body + '</w:tbl>')


def section_break(columns: int) -> str:
    return ('<w:sectPr><w:type w:val="continuous"/>'
            '<w:pgSz w:w="11419" w:h="15621"/>'
            '<w:pgMar w:top="1814" w:right="794" w:bottom="454" w:left="794" '
            'w:header="680" w:footer="454" w:gutter="0"/>'
            f'<w:cols w:num="{columns}" w:space="425"/>'
            '<w:docGrid w:type="lines" w:linePitch="290"/></w:sectPr>')


def image_paragraph(rel_id: str, doc_id: int, filename: str, width_emu: int, height_emu: int) -> str:
    drawing = f'''<w:r><w:drawing><wp:inline distT="0" distB="0" distL="0" distR="0">
<wp:extent cx="{width_emu}" cy="{height_emu}"/><wp:effectExtent l="0" t="0" r="0" b="0"/>
<wp:docPr id="{doc_id}" name="{x(filename)}"/><wp:cNvGraphicFramePr/>
<a:graphic><a:graphicData uri="http://schemas.openxmlformats.org/drawingml/2006/picture">
<pic:pic><pic:nvPicPr><pic:cNvPr id="0" name="{x(filename)}"/><pic:cNvPicPr/></pic:nvPicPr>
<pic:blipFill><a:blip r:embed="{rel_id}"/><a:stretch><a:fillRect/></a:stretch></pic:blipFill>
<pic:spPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="{width_emu}" cy="{height_emu}"/></a:xfrm>
<a:prstGeom prst="rect"><a:avLst/></a:prstGeom></pic:spPr></pic:pic>
</a:graphicData></a:graphic></wp:inline></w:drawing></w:r>'''
    return f'<w:p><w:pPr><w:jc w:val="center"/><w:spacing w:before="80" w:after="40"/></w:pPr>{drawing}</w:p>'


def final_section() -> str:
    return ('<w:sectPr w:rsidR="00CD223E">'
            '<w:headerReference w:type="even" r:id="rId12"/><w:headerReference w:type="default" r:id="rId13"/>'
            '<w:footerReference w:type="even" r:id="rId14"/><w:footerReference w:type="default" r:id="rId15"/>'
            '<w:headerReference w:type="first" r:id="rId16"/><w:footerReference w:type="first" r:id="rId17"/>'
            '<w:type w:val="continuous"/><w:pgSz w:w="11419" w:h="15621"/>'
            '<w:pgMar w:top="1814" w:right="794" w:bottom="454" w:left="794" '
            'w:header="680" w:footer="454" w:gutter="0"/><w:pgNumType w:start="1"/>'
            '<w:cols w:num="2" w:space="425"/><w:titlePg/>'
            '<w:docGrid w:type="lines" w:linePitch="290"/></w:sectPr>')


def manuscript_body(*, include_layout: bool = True) -> str:
    b: list[str] = []
    b.append(paragraph("长走廊退化环境下激光雷达-里程计各向异性融合方法", align="center", size=32,
                       bold=True, east_asia="宋体", before=120, after=100))
    b.append(paragraph("[作者姓名]1，[作者姓名]1+", align="center", size=18, before=20, after=60))
    b.append(paragraph("1. [学校全称] [学院全称]，[省 市 邮编]", align="center", size=16, before=0, after=100))
    b.append(rich_paragraph([("摘  要：", {"bold": True, "east_asia": "黑体"}),
        ("针对无惯性测量单元条件下轮式移动机器人在长直走廊中易出现激光几何约束方向退化、在观测冲突时固定里程计权重难以兼顾稳定性的问题，提出一种面向 Cartographer 2D 的激光雷达-里程计自适应融合方法。首先将多线激光雷达点云投影为二维扫描，利用当前帧点云协方差的主方向和条件数构造平滑退化置信度，并将前端标量平移先验扩展为二维各向异性平方根信息矩阵。其次，结合扫描质量、方向退化信息和 LiDAR-odometry 相对运动残差，对后端 odometry 约束实施可靠性门控下的动态降权。三条独立重复走廊数据的 21 次消融运行表明，所提方向性模块的墙带覆盖率为 0.3111，高于静态低权重与静态高权重组的 0.2456 和 0.2652，且地图画布面积保持在 1383.90 m2。15 次异常注入有效运行中，完整方法相对静态高权重组将 Chamfer 均值由 0.2566 m 降至 0.0805 m、墙体 F1 由 0.4997 提升至 0.8153；正常数据总计 28.33 min 内未发生异常触发。结果支持该方法在已测走廊和确定性一致性异常条件下改善特定结构代理指标并抑制误降权；由于自有数据无连续外部真值，且异常输入不等同于真实轮胎打滑，结论不外推为绝对定位精度或跨场景通用性。", {})],
        size=18, align="both", after=30))
    b.append(rich_paragraph([("关键词：", {"bold": True, "east_asia": "黑体"}),
        ("多线激光雷达；Cartographer 2D；方向性退化；各向异性融合；轮式里程计；一致性异常", {})],
        size=18, align="both", after=50))
    b.append(paragraph("中图分类号：TP242.6    文献标志码：A", align="left", size=16, after=100))
    b.append(paragraph("Anisotropic LiDAR-Odometry Fusion for 2D SLAM in Long-Corridor Degeneracy", align="center", size=22,
                       bold=True, east_asia="Times New Roman", before=60, after=50))
    b.append(paragraph("[AUTHOR NAME]1", align="center", size=16, east_asia="Times New Roman", after=30))
    b.append(paragraph("1. [School and Department], [City Postal Code], China", align="center", size=15,
                       east_asia="Times New Roman", after=50))
    b.append(rich_paragraph([("Abstract: ", {"bold": True, "east_asia": "Times New Roman"}),
        ("A LiDAR-odometry adaptive fusion method for Cartographer 2D is proposed for wheeled robots operating without an inertial measurement unit in long corridors. Multi-line LiDAR points are projected into 2D scans. A smoothed directional-degeneracy confidence is derived from the point-cloud covariance and used to construct a two-dimensional anisotropic square-root information matrix for the front-end translation prior. Scan quality, directional reliability, and LiDAR-odometry relative-motion residuals are then used to gate dynamic down-weighting of back-end odometry constraints. Across 21 ablation runs on three independently recorded corridor bags, the proposed directional module achieved a wall-band coverage of 0.3111, compared with 0.2456 and 0.2652 for the static low- and high-weight baselines. In 15 valid deterministic anomaly-injection runs, the full method reduced mean Chamfer distance from 0.2566 m to 0.0805 m and increased wall F1 from 0.4997 to 0.8153 relative to the static high-weight baseline. No anomaly trigger occurred in 28.33 min of normal recordings. The evidence is limited to structural proxy metrics in the measured corridor and deterministic consistency anomalies; it does not establish absolute localization accuracy, real wheel-slip detection, or cross-scene generalization.", {})],
        size=16, align="both", after=20))
    b.append(rich_paragraph([("Key words: ", {"bold": True, "east_asia": "Times New Roman"}),
        ("multi-line LiDAR; Cartographer 2D; directional degeneracy; anisotropic fusion; wheel odometry; consistency anomaly", {"east_asia": "Times New Roman"})],
        size=16, align="both", after=30))
    if include_layout:
        b.append(paragraph("", section=section_break(1)))

    b.append(paragraph(
        "移动机器人在走廊、电梯厅和开阔连接区之间运行时，环境几何和传感器可靠性会连续变化。长直走廊的平行墙面使二维扫描匹配在走廊纵向缺少足够区分度；轮式里程计虽然可提供短时运动预测，却受轮径标定、航向累计偏置和轮地相对运动影响。若对所有方向和所有时段使用同一组固定权重，低权重会在纵向退化段缺少运动支撑，而高权重又可能将里程计偏置持续传递给位姿图。二维激光 SLAM 已形成粒子滤波、扫描匹配和图优化等主线路径[1-5]，但在无 IMU 的实际平台上，仍需要以较小改动区分退化方向并审慎处理 LiDAR 与 odometry 的观测冲突。",
        first_line=True))
    b.append(paragraph(
        "为解决上述问题，设计一种面向 Cartographer 2D 的方向退化感知与一致性可靠性保护方法。方向性模块利用二维点云协方差的主特征向量构造各向异性平移先验，只在退化主方向增强运动先验；可靠性模块则在 LiDAR 横向与航向约束足够可靠时，使用时间对齐的一致性残差调整对应 odometry 边权重。两个模块并非并列叠加：前者回答哪个方向约束不足，后者回答观测冲突时能否有依据降低某类约束。",
        first_line=True, after=40))

    b.append(heading("1 系统模型与问题定义", 1))
    b.append(heading("1.1 多线点云二维投影与优化对象", 2))
    b.append(paragraph(
        "系统输入为多线三维激光雷达和轮式里程计，不订阅 IMU。每帧点云先依据高度带和量程筛选，再按水平角度栅格保留最小量程，生成标准 LaserScan 并送入 Cartographer 2D。使用三维雷达硬件而输出二维地图并不矛盾：多线回波提供更丰富的环境采样，二维投影则匹配平面导航的状态变量 (x, y, theta) 和占据栅格输出。",
        first_line=True))
    b.append(formula("z_min <= z_i <= z_max,    r_min <= sqrt(x_i^2 + y_i^2) <= r_max", 1))
    b.append(paragraph(
        "Cartographer 前端以预测位姿为初值，联合占据空间匹配代价、平移先验和旋转先验估计局部位姿；后端在位姿图中优化局部 SLAM、回环与 odometry 约束[1,4-6]。现有实现保持占据空间匹配项不变，改动对象为前端平移先验和后端每条 odometry 约束的权重，而不是将点云协方差误称为严格 Hessian 可观性或直接修改 occupied-space 残差。",
        first_line=True, after=30))
    if include_layout:
        b.append(paragraph("", section=section_break(2)))
        b.append(image_paragraph("rId25", 1001, "system_pipeline.png", 6100000, 3760000))
        b.append(paragraph("图1 方向退化感知与一致性可靠性保护总体框架", align="center", size=16, after=0))
        b.append(paragraph("Fig.1 Overall framework of directional degeneracy perception and consistency-aware reliability protection", align="center", size=14, east_asia="Times New Roman", after=20))
        b.append(paragraph("", section=section_break(1)))
    else:
        b.append(paragraph("图1 方向退化感知与一致性可靠性保护总体框架（兼容版未嵌入图片）", align="center", size=16, after=0))
        b.append(paragraph("Fig.1 Overall framework of directional degeneracy perception and consistency-aware reliability protection (image omitted in compatibility version)", align="center", size=14, east_asia="Times New Roman", after=20))

    b.append(heading("2 方向退化感知的各向异性前端融合", 1))
    b.append(heading("2.1 退化主方向与置信度", 2))
    b.append(paragraph(
        "设当前帧滤波后的二维点集为 P={p_i}，其均值和协方差矩阵分别为 p_bar=(1/N) sum p_i 与 Sigma=(1/N) sum(p_i-p_bar)(p_i-p_bar)^T。对 Sigma 作二维特征分解，记最大和最小特征值为 lambda_max、lambda_min，最大特征值对应方向为 v_long，与其正交的方向为 v_lat。条件数 kappa=lambda_max/max(lambda_min, epsilon) 被用作点云整体二阶形状的低开销代理；它仅表示方向性几何不均衡，不等价于扫描匹配 Hessian 的严格可观性。",
        first_line=True))
    b.append(formula("D_k^raw = 1 / {1 + exp[-s(kappa_k - kappa_0)]},    D_k = alpha_D D_k^raw + (1-alpha_D) D_(k-1)", 2))
    b.append(paragraph(
        "使用 Sigmoid 和一阶平滑避免阈值附近的硬切换。由于 v 与 -v 表示同一方向轴，方向滤波前依据与上一帧方向的点积统一符号。实现中先把 tracking 平面中的主方向旋转到 Ceres 平移残差所在的 local-SLAM 坐标系，再构造权重矩阵；该坐标系修正具有 90 度方向单元测试和单包回放验证，避免机器人航向变化时纵向权重错置。",
        first_line=True))
    b.append(heading("2.2 二维各向异性平方根信息矩阵", 2))
    b.append(paragraph(
        "原始平移先验以标量形式同时约束 x、y 两个方向。为按方向独立调权，在特征基中设置纵向和横向权重，并旋转回 local-SLAM 坐标系，得到平方根信息矩阵 S。退化主方向的权重随 D_k 增大，横向权重保持基础值或仅作小幅变化，因此两侧墙面仍可提供横向校正。",
        first_line=True))
    b.append(formula("S_k = R diag(w_long,k, w_lat,k) R^T,    r_k = S_k (t_k - t_k^0)", 3))
    b.append(paragraph(
        "矩阵 S 在代价函数外以 double 类型预计算；Ceres 自动微分路径只对平移误差变量求导。这一实现不增加位姿状态维度，不改变子图、回环或后端图优化的基本结构。",
        first_line=True, after=30))

    b.append(heading("3 一致性异常下的可靠性保护", 1))
    b.append(heading("3.1 观测冲突不等同于真实打滑", 2))
    b.append(paragraph(
        "对相邻节点 i、j，分别计算 LiDAR local SLAM 与 odometry 的相对运动 T_L_ij、T_O_ij，并由 DeltaT_ij=(T_O_ij)^(-1)T_L_ij 提取横向平移残差和航向残差。一致性偏大只表明两种运动估计互相冲突，不能直接归因于轮胎打滑：轮式里程计可能失真，扫描稀疏或几何退化时 LiDAR 匹配也可能失真，双源同时不可靠时则没有足够信息恢复真值。",
        first_line=True))
    b.append(formula("s_ij = w_y |Delta t_y| + w_theta |NormalizeAngle(Delta theta)|", 4))
    b.append(heading("3.2 可靠性门控与逐边动态调权", 2))
    b.append(paragraph(
        "LiDAR 可靠性由有效角度格数量、扫描质量和方向退化信息共同给出。只有当 LiDAR 横向与航向约束达到可靠阈值时，系统才计算有效一致性分数并对该时刻的后端 odometry 边实施快速降权、平滑恢复和迟滞保持；当可靠性不可用或不足时，保持上一权重，避免把 LiDAR 自身失配误判为 odometry 异常。前端与后端异步运行时，退化指标按传感器时间写入有界缓存，并按节点时间查询最近快照，避免历史边使用“最新帧”指标。",
        first_line=True))
    b.append(paragraph(
        "该模块的工程作用是可靠性门控下的鲁棒因子调度，不是无 IMU 条件下的真实轮胎打滑检测器。对于双源同时不可靠的时段，系统仅保守传播并等待观测恢复。",
        first_line=True, after=30))

    b.append(heading("4 实验与结果", 1))
    b.append(heading("4.1 数据、对比与评价边界", 2))
    b.append(paragraph(
        "所有同一组对比均固定点云投影、Cartographer 公共参数和输入 bag，仅改变相应模块开关或权重策略。自有三条走廊重复录制包没有连续外部真值，故只报告可重复的地图结构代理、检测事件和误触发统计，不报告 ATE 或 RPE。确定性异常注入用于验证一致性异常检测和地图保护，不能替代真实轮胎打滑实验。表1列出冻结主证据及其允许的用途。",
        first_line=True, after=20))
    b.append(paragraph("表1 冻结实验数据及评价用途", align="center", size=16, after=0))
    b.append(paragraph("Table 1 Frozen datasets and permitted evaluation use", align="center", size=14, east_asia="Times New Roman", after=0))
    b.append(table(
        ["数据", "运行/时长", "允许评价", "证据边界"],
        [
            ["corridor_repeat_01/02/03", "21 次方向消融；正常段 28.33 min", "墙带覆盖、连续墙段、画布面积、零误触发", "同一物理走廊；无连续外部真值"],
            ["P0 异常注入三包", "15 次有效运行；21 个标注事件", "Chamfer、墙体 F1、异常检测、地图保护", "确定性一致性异常，不等同真实打滑"],
            ["M3DGR Wheel-float01", "4 组已回放", "接入复现与评价链路检查", "Mocap 刚体至车体原点杠杆臂未公开，当前不作性能收益"],
            ["M3DGR Corridor02", "2 组已回放", "走廊结构与运行稳定性", "无 bag 内连续真值，不作 ATE/RPE"],
            ["M3DGR Wheel-float02", "已下载，未运行", "待作为独立公开测试", "本稿不预写任何数值"],
        ], [1600, 1400, 2100, 2400]))
    b.append(heading("4.2 方向性模块消融", 2))
    b.append(paragraph(
        "在三条独立重复走廊包上，每个方法完成 7 次运行，共 21 次且全部成功。Baseline A 为静态各向同性低权重，Baseline B 为静态各向同性高权重，Proposed 为动态二维各向异性权重。表2中 Proposed 的平均墙带覆盖率最高，地图画布面积与静态低权重组接近，明显小于静态高权重组；最长连续墙段不是所有组均最优，因此不将该实验表述为所有地图指标的绝对最优。",
        first_line=True, after=20))
    b.append(paragraph("表2 创新点一三包消融结果（均值 +/- 标准差）", align="center", size=16, after=0))
    b.append(paragraph("Table 2 Ablation of directional fusion over three corridor bags (mean +/- standard deviation)", align="center", size=14, east_asia="Times New Roman", after=0))
    b.append(table(
        ["方法", "画布面积 /m2", "最佳墙带覆盖率", "最长连续墙段 /m", "未知栅格比例"],
        [
            ["Baseline A", "1399.63 +/- 37.63", "0.2456 +/- 0.1064", "9.90 +/- 5.57", "0.8567 +/- 0.0117"],
            ["Baseline B", "2112.71 +/- 642.15", "0.2652 +/- 0.0502", "11.48 +/- 7.47", "0.8982 +/- 0.0325"],
            ["Proposed", "1383.90 +/- 25.04", "0.3111 +/- 0.1001", "11.08 +/- 4.96", "0.8557 +/- 0.0087"],
        ], [1300, 1700, 1700, 1600, 1600]))
    b.append(paragraph(
        "Proposed 组中三包的退化置信度均值分别为 0.7158、0.6712 和 0.4460；在高置信样本上，以在线轨迹 PCA 主轴为代理方向参考，方向误差中位数为 1.542 度、2.542 度和 2.630 度，纵向权重倍率均值为 3.035、2.874 和 2.112。PCA 主轴不是物理走廊实测轴，故该结果支持“方向一致性提高”，而不写成绝对方向测量误差。",
        first_line=True))
    b.append(heading("4.3 一致性异常、门控与地图保护", 2))
    b.append(paragraph(
        "在持续累计 odometry 漂移的 P0 注入条件下，比较静态低权重、静态高权重与启用可靠性门控的完整方法。表3的所有地图数值均以同一 bag 的干净运行作为结构参考，不能解释为对外部真值的轨迹误差。完整方法相对静态高权重组在五项代理指标上均有所改善，说明在该类确定性注入下可减轻高 odometry 权重传入地图的副作用。",
        first_line=True, after=20))
    b.append(paragraph("表3 P0 持续异常注入下的地图保护结果（均值 +/- 标准差）", align="center", size=16, after=0))
    b.append(paragraph("Table 3 Map-protection results under P0 cumulative anomaly injection (mean +/- standard deviation)", align="center", size=14, east_asia="Times New Roman", after=0))
    b.append(table(
        ["方法", "Chamfer均值 /m", "Chamfer P95 /m", "墙体 F1", "主轴角误差 /deg", "轨迹 P95参考偏差 /m"],
        [
            ["Static low", "0.2365 +/- 0.0481", "0.5663 +/- 0.1598", "0.3977 +/- 0.0386", "0.6280 +/- 0.0892", "2.6381 +/- 1.1183"],
            ["Static high", "0.2566 +/- 0.1215", "0.8031 +/- 0.4723", "0.4997 +/- 0.0724", "0.8006 +/- 0.4209", "3.9499 +/- 0.3980"],
            ["Proposed gate", "0.0805 +/- 0.0420", "0.2448 +/- 0.1706", "0.8153 +/- 0.1768", "0.1183 +/- 0.0825", "0.6013 +/- 0.2482"],
        ], [1200, 1450, 1400, 1250, 1400, 1550]))
    b.append(paragraph(
        "21 个标注异常事件中，系统检出 14 个，未出现误报，Precision 为 100.00%，Recall 为 66.67%，F1 为 80.00%；检测延迟均值为 0.484 s，P95 为 0.742 s。正常三包总计 28.33 min 内异常触发为 0，odometry 权重最低值均为 1.0。门控消融中，关闭门控产生 10 个错误异常上升沿，而开启门控为 0；但 Chamfer 均值由 0.0849 m 变为 0.1104 m，表明门控并非所有地图代理均改善，其主要证据是抑制低可靠 LiDAR 条件下的错误降权。",
        first_line=True))
    b.append(heading("4.4 公开数据接入与独立测试计划", 2))
    b.append(paragraph(
        "已完成 M3DGR MID-360 CustomMsg 到 PointCloud2、LaserScan、/odom 以及 Mocap 话题链路的最小接入，并确认公开数据回放未订阅 IMU 或 Avia 话题。Wheel-float01 当前的 Mocap 对齐中，数据包未给出 Mocap 刚体到 base_footprint 原点的显式杠杆臂标定；刚体对齐可掩盖坐标原点不一致，因此不将现有 APE/RPE 数值作为算法收益证据。Corridor02 也没有 bag 内连续真值，只用于走廊结构和 ArUco 辅助评价。已下载的 Wheel-float02 将在参数不再调节的前提下，作为独立公开测试序列运行；运行后才能补入主性能表。",
        first_line=True, after=30))

    b.append(heading("5 讨论", 1))
    b.append(paragraph(
        "方向性模块的作用是以完整二维矩阵替代标量平移先验，避免在横向仍可观时把 odometry 同时强加给两个方向。可靠性模块的作用是对时变冲突实施有条件的后端因子调度，而不是在没有第三信息源时确定故障源。两者联合运行时，创新点一提供 LiDAR 可靠性门控所需的方向信息，创新点二仅在可靠性条件满足时生效；正常段中后者休眠，三包权重保持为 1.0，未观察到对创新点一结果的反向干扰。",
        first_line=True))
    b.append(paragraph(
        "当前证据仍有四项边界。第一，三条自有 bag 来自同一物理走廊，不能代表跨场景泛化。第二，协方差条件数是几何代理，不替代严格 Hessian 可观性。第三，人工一致性异常不能替代真实轮胎打滑的物理验证。第四，Wheel-float01 的刚体标定缺口使现有外部真值对比不能作为最终结论。提交前必须在 Wheel-float02 完成冻结参数的独立测试，并补充可严格定义坐标原点的连续真值评价或收缩为结构代理结论。",
        first_line=True, after=30))

    b.append(heading("6 结论", 1))
    b.append(paragraph(
        "提出了面向无 IMU 多线激光雷达二维 SLAM 的方向退化感知与一致性可靠性保护方法。该方法以点云主方向构造前端各向异性平移先验，并以扫描质量和时间对齐的一致性残差门控后端 odometry 动态权重。冻结实验支持三个有边界的结论：方向性权重已实际进入 Ceres 优化并改善部分走廊结构代理；在确定性累计异常下，动态调权相对静态高权重可减轻地图保护指标的退化；在 28.33 min 正常重复数据中，可靠性门控未触发错误降权。方法尚未证明绝对定位精度、真实打滑识别或跨场景泛化，后续将使用 Wheel-float02 独立测试与严格坐标标定补足证据。",
        first_line=True, after=40))

    b.append(paragraph("参考文献", align="left", size=20, bold=True, east_asia="黑体", before=160, after=20))
    references = [
        "[1] HESS W, KOHLER D, RAPP H, et al. Real-time loop closure in 2D LIDAR SLAM[C]//2016 IEEE International Conference on Robotics and Automation. Stockholm: IEEE, 2016: 1271-1278.",
        "[2] THRUN S, BURGARD W, FOX D. Probabilistic robotics[M]. Cambridge: MIT Press, 2005.",
        "[3] GRISETTI G, STACHNISS C, BURGARD W. Improved techniques for grid mapping with Rao-Blackwellized particle filters[J]. IEEE Transactions on Robotics, 2007, 23(1): 34-46.",
        "[4] KÜMMERLE R, GRISETTI G, STRASDAT H, et al. g2o: A general framework for graph optimization[C]//2011 IEEE International Conference on Robotics and Automation. Shanghai: IEEE, 2011: 3607-3613.",
        "[5] DELLAERT F, KAESS M. Square root SAM: Simultaneous localization and mapping via square root information smoothing[J]. The International Journal of Robotics Research, 2006, 25(12): 1181-1203.",
        "[6] CERES SOLVER TEAM. Ceres Solver[EB/OL]. [2026-07-30]. http://ceres-solver.org/.",
        "[7] CADENA C, CARLONE L, CARRILLO H, et al. Past, present, and future of simultaneous localization and mapping: Toward the robust-perception age[J]. IEEE Transactions on Robotics, 2016, 32(6): 1309-1332.",
        "[8] CENSI A. An ICP variant using a point-to-line metric[C]//2008 IEEE International Conference on Robotics and Automation. Pasadena: IEEE, 2008: 19-25.",
        "[9] SEGAL A, HAEHNEL D, THRUN S. Generalized-ICP[C]//Robotics: Science and Systems. Seattle, 2009.",
        "[10] ZHANG J, SINGH S. LOAM: Lidar odometry and mapping in real-time[C]//Robotics: Science and Systems. Berkeley, 2014.",
    ]
    for reference in references:
        b.append(paragraph(reference, align="both", size=15, first_line=False, after=0))
    b.append(paragraph(
        "注：本冻结证据版保留已核对的基础参考文献。投稿前须完成不少于 20 篇、包含近三年同领域文献的逐篇核验，并将正文引用逐一对齐；作者、单位、基金和通信信息也必须由作者填写后再进入投稿流程。",
        align="both", size=15, italic=True, east_asia="宋体", before=50, after=0))
    return "".join(b) + final_section()


def replace_relationships(xml: bytes, *, include_layout: bool) -> bytes:
    tree = ET.fromstring(xml)
    ns = "{http://schemas.openxmlformats.org/package/2006/relationships}"
    if include_layout:
        ET.SubElement(tree, f"{ns}Relationship", {
            "Id": "rId25",
            "Type": "http://schemas.openxmlformats.org/officeDocument/2006/relationships/image",
            "Target": "media/image3.png",
        })
    return ET.tostring(tree, encoding="utf-8", xml_declaration=True)


def build_document_xml(*, include_layout: bool = True) -> bytes:
    root = (f'<w:document xmlns:wpc="http://schemas.microsoft.com/office/word/2010/wordprocessingCanvas" '
            f'xmlns:mc="http://schemas.openxmlformats.org/markup-compatibility/2006" '
            f'xmlns:o="urn:schemas-microsoft-com:office:office" xmlns:r="{R}" '
            f'xmlns:m="http://schemas.openxmlformats.org/officeDocument/2006/math" '
            f'xmlns:v="urn:schemas-microsoft-com:vml" xmlns:wp14="http://schemas.microsoft.com/office/word/2010/wordprocessingDrawing" '
            f'xmlns:wp="http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing" '
            f'xmlns:w10="urn:schemas-microsoft-com:office:word" xmlns:w="{W}" '
            f'xmlns:w14="http://schemas.microsoft.com/office/word/2010/wordml" '
            f'xmlns:wpg="http://schemas.microsoft.com/office/word/2010/wordprocessingGroup" '
            f'xmlns:wpi="http://schemas.microsoft.com/office/word/2010/wordprocessingInk" '
            f'xmlns:wne="http://schemas.microsoft.com/office/word/2006/wordml" '
            f'xmlns:wps="http://schemas.microsoft.com/office/word/2010/wordprocessingShape" '
            f'xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" '
            f'xmlns:pic="http://schemas.openxmlformats.org/drawingml/2006/picture" mc:Ignorable="w14 wp14">')
    xml = '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>' + root + '<w:body>' + manuscript_body(include_layout=include_layout) + '</w:body></w:document>'
    ET.fromstring(xml.encode("utf-8"))
    return xml.encode("utf-8")


def update_core_properties(xml: bytes) -> bytes:
    tree = ET.fromstring(xml)
    ns = "{http://purl.org/dc/elements/1.1/}"
    title = tree.find(f"{ns}title")
    if title is not None:
        title.text = "长走廊退化环境下激光雷达-里程计各向异性融合方法"
    subject = tree.find(f"{ns}subject")
    if subject is not None:
        subject.text = "冻结证据版投稿初稿"
    return ET.tostring(tree, encoding="utf-8", xml_declaration=True)


def main() -> None:
    if not TEMPLATE.is_file() or not PIPELINE.is_file():
        raise SystemExit("Template or system figure is missing.")
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    def write_docx(output_path: Path, *, include_layout: bool) -> None:
        temp = output_path.with_suffix(".tmp.docx")
        document_xml = build_document_xml(include_layout=include_layout)
        with zipfile.ZipFile(TEMPLATE, "r") as source, zipfile.ZipFile(temp, "w", zipfile.ZIP_DEFLATED) as target:
            for info in source.infolist():
                if info.filename in {"word/document.xml", "word/_rels/document.xml.rels", "docProps/core.xml"}:
                    continue
                # The compatibility copy stays as close as possible to the template.
                if not include_layout and info.filename.startswith("word/media/") and info.filename != "word/media/image2.png":
                    continue
                target.writestr(info, source.read(info.filename))
            target.writestr("word/document.xml", document_xml)
            target.writestr("word/_rels/document.xml.rels", replace_relationships(source.read("word/_rels/document.xml.rels"), include_layout=include_layout))
            target.writestr("docProps/core.xml", update_core_properties(source.read("docProps/core.xml")))
            if include_layout:
                target.write(PIPELINE, "word/media/image3.png")
        shutil.move(temp, output_path)
        with zipfile.ZipFile(output_path, "r") as result:
            bad = result.testzip()
            if bad:
                raise SystemExit(f"Invalid DOCX member: {bad}")
            ET.fromstring(result.read("word/document.xml"))

    write_docx(OUTPUT, include_layout=True)
    write_docx(OUTPUT_OPENABLE, include_layout=False)
    print(OUTPUT)
    print(OUTPUT_OPENABLE)


if __name__ == "__main__":
    main()
