#!/usr/bin/env python3
"""Create a readable comparison from saved OCR results; does not run inference."""
import argparse
import json
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont, ImageOps


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--report', type=Path, required=True)
    p.add_argument('--font', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.output.exists():
        p.error('Use a new output filename')
    d = json.loads(args.report.read_text())
    rows = [r for r in d['rows'] if r['kind']=='recorded_camera' and r['phase']=='after']
    controls = [r for r in d['rows'] if r['kind']=='texture_control']
    before = {r['case']:r for r in d['rows'] if r['phase']=='before'}
    titles = {'lateral':'바깥쪽 시작 배치', 'opposite':'반대쪽 대각선',
              **{f'repeat_{i}':f'같은 배치 재방문 {i}' for i in range(1,4)}}
    font = lambda size: ImageFont.truetype(str(args.font),size)
    board = Image.new('RGB',(1600,1190),'#f4f7fb')
    draw = ImageDraw.Draw(board)
    def text(x,y,s,size=22,color='#21354d'):
        draw.text((x,y),s,font=font(size),fill=color)
    def picture(path,x,y,w,h):
        im=ImageOps.contain(Image.open(path).convert('RGB'),(w,h))
        board.paste(im,(x+(w-im.width)//2,y+(h-im.height)//2))
    text(36,24,'번호판은 검출됐지만, 한글 한 글자를 잘못 읽었습니다',34)
    text(36,78,'정답: 123가4567  |  EasyOCR 1.7.2 · 한국어 모델 · 글자 영역 분리 추가 진단',23)
    text(36,118,'번호판 검출 점수와 OCR 점수는 서로 다릅니다. 아래 수치는 OCR 점수이며, 정답 확률이 아닙니다.',20,'#63758c')
    for x,label in [(36,'실험'),(250,'이동 전 전체 영상'),(600,'이동 후 번호판 crop'),(1030,'판독 결과'),(1280,'OCR 점수'),(1450,'정답 일치')]:
        text(x,170,label,21)
    for i,r in enumerate(rows):
        y=215+i*145
        text(36,y+36,titles.get(r['case'],r['case']),21)
        picture(before[r['case']]['image'],250,y,300,111)
        text(295,y+112,'검출 없음 → OCR 미실행',17,'#63758c')
        picture(r['crop_image'],600,y,380,111)
        text(1030,y+33,r['normalized_text'],28,'#a63831')
        text(1285,y+33,f"{r['recognition_confidence']:.3f}",27)
        text(1460,y+33,'일치' if r['exact_match'] else '불일치',24,'#a63831')
        draw.line((36,y+138,1564,y+138),fill='#d3ddea',width=1)
    y=960
    for r in controls:
        text(36,y,'추가 대조: 깨끗한 원본 텍스처',23)
        picture(r['crop_image'],600,y-15,380,95)
        text(1030,y+5,r['normalized_text'],28,'#a63831')
        text(1285,y+5,f"{r['recognition_confidence']:.3f}",27)
        text(1460,y+5,'불일치',24,'#a63831')
    text(36,1065,'해석: 현재 모델에서 가 → 7 혼동이 남습니다. 이동을 더 해도 해결된다고 단정할 근거는 없습니다.',21)
    text(36,1105,'범위: 같은 합성 번호판 · 이동 후 5장(재방문 3장 포함) · 검출 상자가 그려진 저장 영상.',19,'#63758c')
    text(36,1140,'전체 번호판 crop은 0/5, 글자 영역 분리도 0/5. 기존 Google Vision과 연속 실시간 OCR은 아직 검증하지 않았습니다.',19,'#63758c')
    board.save(args.output)


if __name__=='__main__':
    main()
