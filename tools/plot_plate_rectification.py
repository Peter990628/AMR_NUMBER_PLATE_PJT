#!/usr/bin/env python3
"""Render frozen rectification inputs and OCR results side by side."""
import argparse
import json
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont, ImageOps


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = json.loads(args.report.read_text())
    font_path = Path(__file__).resolve().parents[1]/'local_artifacts/model_comparison_report_2026-11-18/fonts/NanumGothic-Regular.ttf'
    font = ImageFont.truetype(str(font_path), 23)
    small = ImageFont.truetype(str(font_path), 19)
    title = ImageFont.truetype(str(font_path), 34)
    canvas = Image.new('RGB', (1900, 1420), '#f1f5f9')
    draw = ImageDraw.Draw(canvas)
    draw.text((30, 20), '번호판 원근 보정 × PaddleOCR 비교', font=title, fill='#0f172a')
    counts = report['summary']['recorded_camera']['exact_matches']
    draw.text((30, 72), '동일한 카메라 crop 5장 · 정답 123가4567 · 동일 OCR 모델 · 모서리는 OCR과 무관하게 자동 추정', font=font, fill='#334155')
    names = ['original', 'resize_only', 'recrop_resize', 'perspective']
    labels = ['기존 crop', '전체 크기·비율 조정', '영역 재자르기 + 크기 조정', '네 모서리 원근 보정']
    for j, (key, label) in enumerate(zip(names, labels)):
        draw.text((245+j*410, 118), f'{label} ({counts[key]}/5)', font=small, fill='#0f172a')
    rows = sorted(report['rows'], key=lambda row: row['kind']=='texture_control')
    for i, row in enumerate(rows):
        top = 165+i*195
        draw.rounded_rectangle((20, top, 1880, top+183), radius=10, fill='white')
        label = row['id'].replace('_after_0', '').replace('texture_control', '원본 텍스처\n(별도 대조군)')
        draw.text((35, top+20), label, font=font, fill='#0f172a')
        found = row['geometry']['status']=='quad_found'
        draw.text((35, top+85), '모서리 찾음' if found else '모서리 못 찾음\n→ 원본 유지', font=small, fill='#475569')
        for j, name in enumerate(names):
            x = 245+j*410
            path = row['corner_preview'] if name=='original' else row['variants'][name]['image']
            im = Image.open(path).convert('RGB')
            im = ImageOps.contain(im, (385, 100))
            canvas.paste(im, (x+(385-im.width)//2, top+10+(100-im.height)//2))
            result = row['results'][name]
            color = '#166534' if result['exact_match'] else '#b91c1c'
            draw.text((x, top+116), result['raw_text'], font=font, fill=color)
            suffix = ' · 원본 유지' if not found and name in ('recrop_resize', 'perspective') else ''
            draw.text((x, top+149), f"OCR 점수 {result['recognition_confidence']:.3f}{suffix}", font=small, fill='#475569')
    draw.text((30, 1355), '빨간 선/파란 점은 추정 모서리 표시용(OCR 입력 아님). 출력 비율 520:110은 시뮬레이션 번호판 규격을 사용.', font=small, fill='#334155')
    draw.text((30, 1384), '재조정만으로도 5/5 일치: 이 표본에서는 원근 보정만의 추가 효과가 입증되지 않음. 일반 정확도나 이동 성공률이 아님.', font=small, fill='#334155')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(args.output)


if __name__ == '__main__':
    main()
