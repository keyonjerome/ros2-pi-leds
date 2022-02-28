import { NgModule } from '@angular/core';
import { RouterModule, Routes } from '@angular/router';
import { ColorPubComponent } from './color-pub/color-pub.component';

const routes: Routes = [
  {path:'', component:ColorPubComponent}
];

@NgModule({
  imports: [RouterModule.forRoot(routes)],
  exports: [RouterModule]
})
export class AppRoutingModule { }
