import { ComponentFixture, TestBed } from '@angular/core/testing';

import { ColorPubComponent } from './color-pub.component';

describe('ColorPubComponent', () => {
  let component: ColorPubComponent;
  let fixture: ComponentFixture<ColorPubComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ ColorPubComponent ]
    })
    .compileComponents();
  });

  beforeEach(() => {
    fixture = TestBed.createComponent(ColorPubComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
